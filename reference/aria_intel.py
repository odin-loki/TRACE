from __future__ import annotations
import numpy as np
from numpy.linalg import inv, cholesky, solve, det
from scipy.stats import chi2
from scipy.special import gammaln
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Callable, Any
import uuid, time
from abc import ABC, abstractmethod
from collections import defaultdict

STATE_DIM      = 4
OBS_DIM        = 2
N_PARTICLES    = 320
MC_SAMPLES     = 600
REFIT_INTERVAL = 5
GIBBS_SWEEPS   = 14

MOU_MODELS = {
    'foot':       {'theta': 0.30, 'sigma': 2.0},
    'vehicle':    {'theta': 0.10, 'sigma': 8.0},
    'stationary': {'theta': 2.00, 'sigma': 0.5},
    'fast':       {'theta': 0.05, 'sigma': 15.0},
}
MODEL_KEYS = list(MOU_MODELS.keys())
N_MODELS   = len(MODEL_KEYS)

_DT          = 1.0
_MOU_ALPHA   = np.array([np.exp(-MOU_MODELS[k]['theta']*_DT) for k in MODEL_KEYS])
_MOU_SIG_V   = np.array([
    MOU_MODELS[k]['sigma'] * np.sqrt((1.0 - np.exp(-2*MOU_MODELS[k]['theta']*_DT)) /
                                      (2*MOU_MODELS[k]['theta']))
    for k in MODEL_KEYS
])
_MOU_SS_VVAR = np.array([MOU_MODELS[k]['sigma']**2 / (2.0*MOU_MODELS[k]['theta'])
                          for k in MODEL_KEYS])

MODEL_TRANS = np.array([
    [0.85, 0.10, 0.04, 0.01],
    [0.05, 0.88, 0.02, 0.05],
    [0.15, 0.05, 0.78, 0.02],
    [0.02, 0.20, 0.01, 0.77],
])
H = np.array([[1,0,0,0],[0,1,0,0]], dtype=np.float64)
R = np.diag([25.0, 25.0])

P_DETECTION      = 0.85
P_SURVIVAL       = 0.995
GATE_THRESH      = float(chi2.ppf(0.999, df=2))
R_BIRTH          = 0.65
R_PRUNE          = 0.05
R_CONFIRM        = 0.55
R_DORMANT_THRESH = 0.04
DORMANT_TIMEOUT  = 40
DECEPTION_DECAY  = 0.98
MOU_POS_JITTER   = 1.5
TM_VEL_NOISE     = 10.0
POSS_ALPHA       = 0.25
GROUP_SPAWN_MRATE_THRESH = 0.85
GROUP_SPAWN_VVAR_THRESH  = 12.0

MODALITY_WEIGHT = {'GEOINT':0.95,'SIGINT':0.82,'COMMS':0.75,'HUMINT':0.65,'OSINT':0.55}
TIER = {0.82:'IMMEDIATE', 0.62:'HIGH', 0.42:'MEDIUM', 0.22:'LOW'}

_INV_R   = inv(R)
_LOG_DET = float(np.log(det(R)))
_2PI_D   = float(OBS_DIM * np.log(2*np.pi))
_INV_RV  = inv(np.diag([TM_VEL_NOISE**2, TM_VEL_NOISE**2]))


def logsumexp(a, axis=None, keepdims=False):
    a = np.asarray(a, dtype=np.float64)
    a_max = np.max(a, axis=axis, keepdims=True)
    out = np.log(np.sum(np.exp(a - a_max), axis=axis, keepdims=keepdims))
    if keepdims:
        return out + a_max
    sq = np.squeeze(a_max, axis=axis) if axis is not None else a_max.flat[0]
    return out + sq


def _chol_logpdf(X, mu, cov):
    D = mu.shape[0]
    X = np.atleast_2d(X)
    L = cholesky(cov + np.eye(D)*1e-7)
    d = X - mu
    y = solve(L, d.T).T
    ld = 2.0 * np.sum(np.log(np.diag(L)))
    return -0.5*(np.sum(y**2, axis=1) + ld + D*np.log(2*np.pi))


def _betweenness_centrality(adj: np.ndarray) -> np.ndarray:
    n = adj.shape[0]
    bc = np.zeros(n)
    for s in range(n):
        sigma = np.zeros(n); sigma[s] = 1.0
        d = np.full(n, -1, dtype=int); d[s] = 0
        P = [[] for _ in range(n)]
        queue = [s]; stack = []
        while queue:
            v = queue.pop(0); stack.append(v)
            for w in range(n):
                if adj[v, w] <= 0: continue
                if d[w] < 0:
                    queue.append(w); d[w] = d[v] + 1
                if d[w] == d[v] + 1:
                    sigma[w] += sigma[v]; P[w].append(v)
        delta = np.zeros(n)
        while stack:
            w = stack.pop()
            for v in P[w]:
                if sigma[w] > 0:
                    delta[v] += sigma[v]/sigma[w] * (1.0 + delta[w])
            if w != s:
                bc[w] += delta[w]
    denom = max((n-1)*(n-2)/2.0, 1.0)
    return bc / denom


@dataclass
class Observation:
    obs_id:     str
    timestamp:  float
    position:   Optional[np.ndarray]
    modality:   str
    confidence: float = 1.0
    source_id:  str   = ''

    @property
    def weight(self) -> float:
        return MODALITY_WEIGHT.get(self.modality, 0.5) * self.confidence


_LOG_2PI = float(np.log(2*np.pi))


def _fwdsub3(L: np.ndarray, B: np.ndarray) -> np.ndarray:
    y = np.empty_like(B)
    y[0] =  B[0] / L[0,0]
    y[1] = (B[1] - L[1,0]*y[0]) / L[1,1]
    y[2] = (B[2] - L[2,0]*y[0] - L[2,1]*y[1]) / L[2,2]
    return y


def _gmm_logpdf_batch(X: np.ndarray, means: np.ndarray, Ls: np.ndarray,
                       log_dets: np.ndarray, log_pi: np.ndarray) -> np.ndarray:
    N, D = X.shape; K = len(log_pi)
    out = np.empty((N, K))
    c = -0.5*(D*_LOG_2PI)
    for k in range(K):
        diff = (X - means[k]).T
        y = _fwdsub3(Ls[k], diff)
        out[:, k] = c - 0.5*(y[0]*y[0] + y[1]*y[1] + y[2]*y[2] + log_dets[k]) + log_pi[k]
    return out


class PatternOfLife:
    def __init__(self, K: int = 5, min_obs: int = 15):
        self.K = K; self.min_obs = min_obs
        self.obs: List[np.ndarray] = []
        self.means = self.covs = self.pi = None
        self._Ls: Optional[np.ndarray] = None
        self._log_dets: Optional[np.ndarray] = None
        self._log_pi: Optional[np.ndarray] = None
        self.baseline_nll = 4.0; self._fitted = False
        self._obs_since_refit = 0

    def _rebuild_cache(self):
        K, D = self.means.shape[0], self.means.shape[1]
        Ls = np.zeros((K, D, D))
        ld = np.zeros(K)
        for k in range(K):
            L = cholesky(self.covs[k] + np.eye(D)*1e-6)
            Ls[k] = L; ld[k] = 2.0*np.sum(np.log(np.diag(L)))
        self._Ls = Ls; self._log_dets = ld
        self._log_pi = np.log(self.pi + 1e-300)

    def add(self, t: float, pos: np.ndarray):
        self.obs.append(np.array([(t % 86400)/3600.0, pos[0], pos[1]]))
        self._obs_since_refit += 1
        if len(self.obs) >= self.min_obs and self._obs_since_refit >= REFIT_INTERVAL:
            self._em_fit(); self._obs_since_refit = 0

    def clone_from(self, other: 'PatternOfLife'):
        self.obs = list(other.obs)
        self.means = other.means.copy() if other.means is not None else None
        self.covs  = other.covs.copy()  if other.covs  is not None else None
        self.pi    = other.pi.copy()    if other.pi    is not None else None
        self._Ls   = other._Ls.copy()   if other._Ls   is not None else None
        self._log_dets = other._log_dets.copy() if other._log_dets is not None else None
        self._log_pi   = other._log_pi.copy()   if other._log_pi   is not None else None
        self.baseline_nll = other.baseline_nll
        self._fitted = other._fitted; self._obs_since_refit = 0

    def _em_fit(self):
        data = np.array(self.obs[-300:])
        N, D = data.shape
        K = min(self.K, N//3)
        if K < 1: return
        centres = [data[np.random.randint(N)].copy()]
        for _ in range(K-1):
            dists = np.array([min(float(np.sum((x-c)**2)) for c in centres) for x in data])
            dists /= dists.sum()+1e-300
            centres.append(data[np.random.choice(N, p=dists)].copy())
        means = np.array(centres)
        covs  = np.array([np.diag([1.0,500.0,500.0])]*K, dtype=np.float64)
        pi    = np.ones(K)/K
        reg   = np.eye(D)*5e-3
        Ls    = np.zeros((K,D,D)); ld = np.zeros(K)
        for k in range(K):
            L = cholesky(covs[k]+np.eye(D)*1e-6); Ls[k]=L; ld[k]=2*np.sum(np.log(np.diag(L)))
        log_pi = np.log(pi+1e-300)
        for it in range(35):
            log_r = _gmm_logpdf_batch(data, means, Ls, ld, log_pi)
            log_norm = logsumexp(log_r, axis=1, keepdims=True)
            r = np.exp(log_r - log_norm)
            Nk = r.sum(0)+1e-6; pi = Nk/N; log_pi = np.log(pi+1e-300)
            means = (r.T@data)/Nk[:,None]
            for k in range(K):
                d = data - means[k]
                covs[k] = (r[:,k:k+1]*d).T@d/Nk[k] + reg
                if it % 5 == 4 or it == 34:
                    L = cholesky(covs[k]+np.eye(D)*1e-6); Ls[k]=L; ld[k]=2*np.sum(np.log(np.diag(L)))
        self.means = means; self.covs = covs; self.pi = pi
        self._Ls = Ls; self._log_dets = ld; self._log_pi = log_pi
        self._fitted = True
        tail = data[-40:]
        lps = logsumexp(_gmm_logpdf_batch(tail, means, Ls, ld, log_pi), axis=1)
        self.baseline_nll = float(-np.mean(lps))

    def _log_p_batch(self, X: np.ndarray) -> np.ndarray:
        lp = _gmm_logpdf_batch(X, self.means, self._Ls, self._log_dets, self._log_pi)
        return logsumexp(lp, axis=1)

    def _log_p(self, x):
        if not self._fitted: return -4.0
        return float(self._log_p_batch(x.reshape(1,-1))[0])

    def anomaly_score(self, t: float, pos: np.ndarray) -> float:
        if not self._fitted: return 0.5
        nll = -self._log_p(np.array([(t%86400)/3600.0, pos[0], pos[1]]))
        x = (nll - self.baseline_nll)/max(abs(self.baseline_nll), 1.0)
        return float(np.clip(1/(1+np.exp(-x)), 0, 1))

    def predict_location(self, t: float, n_mc: int = 60) -> Tuple[np.ndarray, float]:
        if not self._fitted: return np.zeros(2), 999.0
        K = len(self.pi); hour = (t%86400)/3600.0
        hour_pts = np.column_stack([np.full(K,hour), self.means[:,1], self.means[:,2]])
        time_lw  = self._log_p_batch(hour_pts) + self._log_pi
        time_w   = np.exp(time_lw - logsumexp(time_lw))
        time_w   = time_w * self.pi; time_w /= time_w.sum()+1e-300
        ks = np.random.choice(K, n_mc, p=time_w)
        counts = np.bincount(ks, minlength=K)
        samples = np.zeros((n_mc, 3))
        idx = 0
        for k in range(K):
            c = counts[k]
            if c == 0: continue
            L = self._Ls[k]
            z = np.random.randn(3, c)
            samples[idx:idx+c] = self.means[k] + (L @ z).T
            idx += c
        pos2d = samples[:, 1:3]
        return pos2d.mean(0), float(np.std(np.linalg.norm(pos2d-pos2d.mean(0), axis=1)))

    def active_windows(self) -> List[Tuple[float,float]]:
        if not self._fitted: return []
        out = []
        for k in range(len(self.pi)):
            if self.pi[k] > 1.0/len(self.pi)*0.5:
                ch = float(self.means[k,0]); sh = float(np.sqrt(self.covs[k,0,0]))
                out.append((round(ch-sh,1), round(ch+sh,1)))
        return out


class MOUParticleFilter:
    _inv_R   = _INV_R
    _log_det = _LOG_DET
    _2pi_d   = _2PI_D
    _inv_Rv  = _INV_RV

    def __init__(self, n: int = N_PARTICLES):
        self.n = n
        self.X: Optional[np.ndarray] = None
        self.w: Optional[np.ndarray] = None
        self.mu = np.ones(N_MODELS)/N_MODELS
        self._cache_valid = False
        self._pos_c = self._mean_c = self._P_c = self._S_c = self._Si_c = None

    def _inv(self): self._cache_valid = False; self._pos_c = self._mean_c = self._P_c = self._S_c = self._Si_c = None

    def _ec(self):
        if self._cache_valid: return
        self._pos_c  = self.w @ self.X[:,:2]
        self._mean_c = self.w @ self.X
        d = self.X - self._mean_c
        self._P_c  = (self.w[:,None]*d).T @ d
        self._S_c  = H @ self._P_c @ H.T + R
        try: self._Si_c = inv(self._S_c)
        except: self._Si_c = np.eye(OBS_DIM)/25.0
        self._cache_valid = True

    def init(self, pos: np.ndarray, pos_s: float = 35.0):
        self.X = np.zeros((self.n, STATE_DIM))
        self.X[:,0] = pos[0] + np.random.normal(0, pos_s, self.n)
        self.X[:,1] = pos[1] + np.random.normal(0, pos_s, self.n)
        mode_ss_v = np.sqrt(np.dot(np.ones(N_MODELS)/N_MODELS, _MOU_SS_VVAR))
        self.X[:,2] = np.random.normal(0, mode_ss_v, self.n)
        self.X[:,3] = np.random.normal(0, mode_ss_v, self.n)
        self.w = np.ones(self.n)/self.n
        self.mu = np.ones(N_MODELS)/N_MODELS
        self._inv()

    def predict(self):
        new_mu = MODEL_TRANS.T @ self.mu; new_mu /= new_mu.sum()
        midx = np.random.choice(N_MODELS, size=self.n, p=new_mu)
        alpha_p = _MOU_ALPHA[midx]; sig_p = _MOU_SIG_V[midx]
        v = self.X[:,2:4]
        eps_v = np.random.randn(self.n, 2)
        v_new = alpha_p[:,None]*v + sig_p[:,None]*eps_v
        pos_jitter = np.random.randn(self.n, 2) * MOU_POS_JITTER
        self.X = np.column_stack([
            self.X[:,:2] + _DT*(v + v_new)/2.0 + pos_jitter,
            v_new
        ])
        self.mu = new_mu; self._inv()

    def update(self, obs: np.ndarray, r_scale: float = 1.0):
        diff  = obs[None] - (H @ self.X.T).T
        inv_R = self._inv_R / r_scale
        maha2 = np.einsum('ni,ij,nj->n', diff, inv_R, diff)
        log_w = -0.5*(maha2 + self._log_det + np.log(r_scale) + self._2pi_d)
        log_w += np.log(self.w+1e-300); log_w -= logsumexp(log_w)
        self.w = np.exp(log_w); self.w /= self.w.sum()
        if 1.0/(np.sum(self.w**2)+1e-300) < self.n*0.4: self._resample()
        self._inv()

    def update_trajectory(self, obs_curr: np.ndarray, obs_prev: np.ndarray):
        v_obs = (obs_curr - obs_prev) / _DT
        diff_v = v_obs[None] - self.X[:,2:4]
        maha2_v = np.einsum('ni,ij,nj->n', diff_v, self._inv_Rv, diff_v)
        log_w = np.log(self.w+1e-300) - 0.5*maha2_v
        log_w -= logsumexp(log_w); self.w = np.exp(log_w); self.w /= self.w.sum()
        if 1.0/(np.sum(self.w**2)+1e-300) < self.n*0.35: self._resample()
        self._inv()

    def _resample(self):
        u = (np.random.random()+np.arange(self.n))/self.n
        idx = np.clip(np.searchsorted(np.cumsum(self.w), u), 0, self.n-1)
        self.X = self.X[idx].copy(); self.w = np.ones(self.n)/self.n

    @property
    def pos(self): self._ec(); return self._pos_c
    @property
    def vel(self): return self.w @ self.X[:,2:]
    @property
    def mean(self): self._ec(); return self._mean_c
    @property
    def dominant_model(self): return MODEL_KEYS[int(np.argmax(self.mu))]

    def covariance(self): self._ec(); return self._P_c
    def innovation_cov(self): self._ec(); return self._S_c
    def mahal2(self, obs: np.ndarray): self._ec(); inn = obs - self._pos_c; return float(inn @ self._Si_c @ inn)
    def pos_uncertainty(self): self._ec(); return float(np.sqrt(np.trace(self._P_c[:2,:2])))


class BernoulliTrack:
    _counter = 0

    def __init__(self, r: float, pf: MOUParticleFilter, t0: float = 0.0,
                 parent_id: Optional[str] = None):
        BernoulliTrack._counter += 1
        self.tid = f"T{BernoulliTrack._counter:04d}"
        self.r   = float(np.clip(r, 0, 1))
        self.pi_r = float(r)
        self.pf  = pf
        self.pol = PatternOfLife()
        self.born_at = t0; self.last_seen = t0
        self.age = 0; self.n_hit = 0; self.n_miss = 0
        self.parent_id = parent_id
        self._obs_weights: List[float] = []
        self._threat_ema = 0.5; self._threat_persistence = 0
        self._pos_history: List[np.ndarray] = []
        self._vel_history: List[np.ndarray] = []
        self._ts_history:  List[float] = []
        self._threat_history: List[float] = []
        self._last_obs_pos: Optional[np.ndarray] = None
        self._last_obs_ts: float = -999.0
        self.mrate: float = 0.0
        self.poss_mismatch: float = 0.0

    def predict(self):
        self.r = self.r * P_SURVIVAL
        self.pi_r = self.pi_r * P_SURVIVAL
        self.pf.predict(); self.age += 1

    def update_hit(self, obs: Observation):
        use_tm = (self._last_obs_pos is not None and
                  abs(obs.timestamp - self._last_obs_ts - _DT) < 0.1)
        self.pf.update(obs.position, r_scale=1.0/(obs.weight+0.1))
        if use_tm:
            self.pf.update_trajectory(obs.position, self._last_obs_pos)

        L = P_DETECTION; c = 1e-4
        self.r = float(np.clip(self.r*L/(self.r*L+(1-self.r)*c+1e-300), 0, 0.9999))

        pi_L = min(1.0, obs.weight * P_DETECTION)
        self.pi_r = float(np.clip(max(self.pi_r*pi_L, POSS_ALPHA*self.pi_r), 0, 1))

        prob_p = self.r; poss_p = self.pi_r
        self.poss_mismatch = float(abs(prob_p - poss_p) / (max(prob_p, poss_p)+1e-6))

        self.last_seen = obs.timestamp; self.n_hit += 1
        self._obs_weights.append(obs.weight)
        self.pol.add(obs.timestamp, obs.position)
        self._pos_history.append(self.pf.pos.copy())
        self._vel_history.append(self.pf.vel.copy())
        self._ts_history.append(obs.timestamp)
        if len(self._pos_history) > 50:
            self._pos_history.pop(0); self._vel_history.pop(0); self._ts_history.pop(0)
        self._last_obs_pos = obs.position.copy(); self._last_obs_ts = obs.timestamp
        self.mrate = self.n_hit / max(self.age, 1)

    def update_miss(self):
        L = 1 - P_DETECTION
        self.r = float(np.clip(self.r*L/(self.r*L+(1-self.r)+1e-300), 0, 1))
        self.pi_r = float(self.pi_r * (1 - P_DETECTION * POSS_ALPHA))
        self.n_miss += 1

    def update_threat_ema(self, score: float):
        alpha = 0.3
        self._threat_ema = alpha*score + (1-alpha)*self._threat_ema
        self._threat_history.append(score)
        if len(self._threat_history) > 20: self._threat_history.pop(0)
        if score > 0.62: self._threat_persistence += 1
        else: self._threat_persistence = max(0, self._threat_persistence-1)

    @property
    def pos(self): return self.pf.pos
    @property
    def vel(self): return self.pf.vel
    @property
    def detection_density(self): return min(1.0, self.n_hit/max(self.age,1))
    @property
    def mean_obs_quality(self): return float(np.mean(self._obs_weights)) if self._obs_weights else 0.5

    def pos_uncertainty(self): return self.pf.pos_uncertainty()


class AdaptiveClutterEstimator:
    def __init__(self):
        self.alpha = 3.0; self.beta = 1.0; self._win: List[int] = []

    def update(self, n: int):
        self._win.append(n)
        if len(self._win) > 20: self._win.pop(0)
        self.alpha = 3.0 + sum(self._win); self.beta = 1.0 + len(self._win)

    @property
    def rate(self): return self.alpha / self.beta
    def density(self, vol): return self.rate / vol


class SourceCredibilityTracker:
    def __init__(self):
        self._scores: Dict[str,float] = {}; self._counts: Dict[str,int] = {}

    def update(self, sid: str, obs_ll: float, thresh: float):
        if sid not in self._scores: self._scores[sid] = 0.8
        ok = obs_ll > thresh
        self._scores[sid] = DECEPTION_DECAY*self._scores[sid] + (1-DECEPTION_DECAY)*(0.95 if ok else 0.1)
        self._counts[sid] = self._counts.get(sid,0)+1

    def get(self, sid: str): return self._scores.get(sid, 0.8)


class GibbsAssigner:
    def __init__(self, sweeps: int = GIBBS_SWEEPS):
        self.sweeps = sweeps

    def assign(self, tracks: List[BernoulliTrack], observations: List[Observation]) -> Dict[int,int]:
        if not tracks or not observations: return {}
        n_t, n_o = len(tracks), len(observations)
        log_like = np.full((n_t, n_o), -np.inf)
        for i, t in enumerate(tracks):
            for j, o in enumerate(observations):
                if o.position is None: continue
                m2 = t.pf.mahal2(o.position)
                if m2 < GATE_THRESH:
                    log_like[i,j] = -0.5*m2 + np.log(o.weight+1e-300)
        clutter_ll = np.log(1e-5)
        asgn = np.full(n_t, -1, dtype=int)
        for i in range(n_t):
            valid = np.where(log_like[i] > -np.inf)[0]
            if len(valid): asgn[i] = valid[int(np.argmax(log_like[i,valid]))]
        for _ in range(self.sweeps):
            for i in np.random.permutation(n_t):
                vj = np.where(log_like[i] > -np.inf)[0]
                if not len(vj): asgn[i] = -1; continue
                conflict = 0.3 * np.array([int(np.sum(asgn[:i]==j)+np.sum(asgn[i+1:]==j)) for j in vj])
                cll = np.concatenate([[clutter_ll], log_like[i,vj]-conflict])
                cll -= logsumexp(cll); probs = np.exp(cll)
                ch = int(np.random.choice(len(cll), p=probs))
                asgn[i] = -1 if ch == 0 else int(vj[ch-1])
        return {i: int(asgn[i]) for i in range(n_t) if asgn[i] >= 0}


class PMBMManager:
    def __init__(self, area: Tuple[float,float,float,float]):
        self.area = area
        self.area_vol = (area[1]-area[0])*(area[3]-area[2])
        self.tracks:  List[BernoulliTrack] = []
        self.dormant: List[Tuple[BernoulliTrack,int]] = []
        self.clutter  = AdaptiveClutterEstimator()
        self.gibbs    = GibbsAssigner()
        self.cred     = SourceCredibilityTracker()
        self.scan = 0

    def predict(self):
        for t in self.tracks: t.predict()
        self.scan += 1

    def _check_reacquisition(self, obs: Observation, ts: float) -> Optional[BernoulliTrack]:
        if not self.dormant or obs.position is None: return None
        best_s, best_i = -np.inf, -1
        for idx, (dt_, _) in enumerate(self.dormant):
            if not dt_.pol._fitted: continue
            pred_pos, pred_unc = dt_.pol.predict_location(ts)
            dist = float(np.linalg.norm(obs.position - pred_pos))
            if dist > max(pred_unc*3, 200.0): continue
            s = -dist/max(pred_unc,1)
            if s > best_s: best_s = s; best_i = idx
        if best_i < 0: return None
        dt_track, _ = self.dormant.pop(best_i)
        dt_track.pf.init(obs.position); dt_track.r = R_BIRTH; dt_track.last_seen = ts
        return dt_track

    def _try_group_spawn(self, new_t: BernoulliTrack, ts: float):
        for ex in self.tracks:
            if ex.tid == new_t.tid: continue
            sep = float(np.linalg.norm(new_t.pos - ex.pos))
            if sep > 80.0: continue
            vvar = float(np.var(np.linalg.norm(ex.pf.X[:,2:] - ex.pf.vel, axis=1)))
            is_group = (ex.mrate > GROUP_SPAWN_MRATE_THRESH and
                        vvar > GROUP_SPAWN_VVAR_THRESH and ex.age > 5)
            if is_group:
                new_t.parent_id = ex.tid
                pol_c = PatternOfLife(); pol_c.clone_from(ex.pol)
                new_t.pol = pol_c; break
            elif sep < 60.0 and ex.age > 3:
                new_t.parent_id = ex.tid; break

    def update(self, observations: List[Observation], ts: float):
        valid = [o for o in observations if o.position is not None]
        if not valid:
            for t in self.tracks: t.update_miss()
            self._prune(ts); return

        asgn = self.gibbs.assign(self.tracks, valid)
        assigned_obs = set(asgn.values())
        self.clutter.update(max(0, len(valid)-len(assigned_obs)))
        cd = self.clutter.density(self.area_vol)

        for i, track in enumerate(self.tracks):
            j = asgn.get(i, -1)
            if j >= 0:
                obs = valid[j]
                obs_ll = -0.5*track.pf.mahal2(obs.position)
                if obs.source_id: self.cred.update(obs.source_id, obs_ll, -5.0)
                trust = self.cred.get(obs.source_id)
                adj = Observation(obs.obs_id, obs.timestamp, obs.position,
                                  obs.modality, obs.confidence*trust, obs.source_id)
                L = P_DETECTION
                track.r = float(np.clip(track.r*L/(track.r*L+(1-track.r)*cd+1e-300), 0, 0.9999))
                track.update_hit(adj)
            else:
                track.update_miss()

        for j, obs in enumerate(valid):
            if j in assigned_obs: continue
            if obs.weight * self.cred.get(obs.source_id) < 0.25: continue
            reacq = self._check_reacquisition(obs, ts)
            if reacq is not None:
                reacq.update_hit(obs); self.tracks.append(reacq); continue
            if obs.weight > 0.25:
                pf = MOUParticleFilter(N_PARTICLES); pf.init(obs.position)
                nt = BernoulliTrack(R_BIRTH, pf, t0=ts)
                nt.update_hit(obs); self._try_group_spawn(nt, ts); self.tracks.append(nt)

        self._prune(ts)

    def _prune(self, ts: float):
        remaining = []
        for t in self.tracks:
            if t.r > R_PRUNE: remaining.append(t)
            elif t.r > R_DORMANT_THRESH and t.pol._fitted: self.dormant.append((t, self.scan))
        self.tracks = remaining
        if len(self.tracks) > 80:
            self.tracks.sort(key=lambda t: -t.r); self.tracks = self.tracks[:80]
        self.dormant = [(t,s) for t,s in self.dormant if self.scan-s < DORMANT_TIMEOUT]

    def confirmed(self, thresh: float = R_CONFIRM) -> List[BernoulliTrack]:
        return sorted([t for t in self.tracks if t.r >= thresh], key=lambda t: -t.r)


def _priority(s: float) -> str:
    for thresh, label in sorted(TIER.items(), reverse=True):
        if s >= thresh: return label
    return 'MONITOR'


def score_track(track: BernoulliTrack, ts: float, hvls: List[np.ndarray],
                hvl_radius: float, n_mc: int = MC_SAMPLES) -> Dict:
    pos = track.pos
    pol  = track.pol.anomaly_score(ts, pos) if track.pol._fitted else 0.5
    dd   = track.detection_density
    hvl  = float(max(np.exp(-np.linalg.norm(pos-h)/hvl_radius) for h in hvls)) if hvls else 0.0
    vel_mag = float(np.linalg.norm(track.vel))
    motion  = float(np.clip(vel_mag/30.0, 0, 1))
    persist = float(np.clip(track._threat_persistence/10.0, 0, 1))
    ema_adj = float(np.clip(track._threat_ema, 0, 1))
    mismatch_penalty = float(np.clip(1.0 - track.poss_mismatch, 0, 1))

    a = np.array([track.r*20+1, pol*8+1, dd*8+1, hvl*8+1,
                  motion*6+1, persist*6+1, ema_adj*8+1, mismatch_penalty*4+1])
    b = np.array([(1-track.r)*20+1, (1-pol)*8+1, (1-dd)*8+1, (1-hvl)*8+1,
                  (1-motion)*6+1, (1-persist)*6+1, (1-ema_adj)*8+1, (1-mismatch_penalty)*4+1])
    weights = np.array([0.23, 0.18, 0.13, 0.13, 0.08, 0.08, 0.10, 0.07])
    n_mc_use = min(n_mc, 250)
    samps   = np.random.beta(a[:,None], b[:,None], size=(8, n_mc_use))
    scores  = weights @ samps

    mean_s = float(scores.mean())
    track.update_threat_ema(mean_s)

    return {
        'threat_score_mean': round(mean_s,4),
        'threat_score_std':  round(float(scores.std()),4),
        'threat_score_p90':  round(float(np.percentile(scores,90)),4),
        'threat_score_p95':  round(float(np.percentile(scores,95)),4),
        'priority':          _priority(mean_s),
        'threat_ema':        round(track._threat_ema,4),
        'threat_persistence': track._threat_persistence,
        'dominant_model':    track.pf.dominant_model,
        'poss_mismatch':     round(track.poss_mismatch,4),
        'breakdown': {
            'existence':     round(track.r,4),
            'poss_exist':    round(track.pi_r,4),
            'pol_anomaly':   round(pol,4),
            'det_density':   round(dd,4),
            'hvl_proximity': round(hvl,4),
            'motion_score':  round(motion,4),
            'persistence':   round(persist,4),
        }
    }


class TradecraftDetector:
    def __init__(self):
        self._visits: Dict[str, List[Tuple[float,np.ndarray]]] = defaultdict(list)
        self._rv_hist: Dict[tuple,int] = defaultdict(int)

    def _winding(self, positions):
        if len(positions) < 6: return 0.0
        pts = np.array(positions); c = pts.mean(0)
        angles = np.arctan2(pts[:,1]-c[1], pts[:,0]-c[0])
        return abs(float(np.diff(np.unwrap(angles)).sum()))/(2*np.pi)

    def detect(self, tracks: List[BernoulliTrack], ts: float, scan: int) -> List[Dict]:
        events = []
        for t in tracks:
            if t.pol._fitted:
                self._visits[t.tid].append((ts, t.pos.copy()))
                if len(self._visits[t.tid]) > 40: self._visits[t.tid].pop(0)

        for i in range(len(tracks)):
            for j in range(i+1, len(tracks)):
                ta, tb = tracks[i], tracks[j]
                sep = float(np.linalg.norm(ta.pos - tb.pos))
                key = (ta.tid, tb.tid)
                if sep < 60.0:
                    self._rv_hist[key] = self._rv_hist.get(key, 0) + 1
                    if self._rv_hist[key] == 1:
                        events.append({'type':'BRUSH_PASS','tracks':[ta.tid,tb.tid],
                                       'sep_m':round(sep,1),'timestamp':ts,'severity':'HIGH'})
                else:
                    if self._rv_hist.get(key, 0) >= 1: self._rv_hist[key] = 0

        for t in tracks:
            vis = self._visits.get(t.tid, [])
            if len(vis) < 8: continue
            pts = np.array([v[1] for v in vis[-12:]])
            c = pts.mean(0); d = pts - c
            angles = np.arctan2(d[:,1], d[:,0])
            wn = abs(np.diff(np.unwrap(angles)).sum())/(2*np.pi)
            if wn >= 0.65:
                events.append({'type':'SDR_PATTERN','track':t.tid,
                               'winding_number':round(float(wn),2),'timestamp':ts,'severity':'HIGH'})

        if len(tracks) >= 2 and scan % 3 == 0:
            loc_vis: Dict[str, List[Tuple[str,float]]] = defaultdict(list)
            for t in tracks:
                for vts, vpos in self._visits.get(t.tid, [])[-5:]:
                    cell = f"{int(vpos[0]/200)}_{int(vpos[1]/200)}"
                    loc_vis[cell].append((t.tid, vts))
            for cell, visitors in loc_vis.items():
                tids = list(set(v[0] for v in visitors))
                if len(tids) < 2: continue
                times = [v[1] for v in visitors]
                spread = max(times)-min(times)
                if 60 < spread < 1800:
                    simult = sum(1 for a,at in visitors for b,bt in visitors
                                 if a!=b and abs(at-bt)<30)
                    if simult == 0:
                        events.append({'type':'DEAD_DROP','tracks':tids[:3],'cell':cell,
                                       'time_spread_s':round(spread,0),'timestamp':ts,'severity':'IMMEDIATE'})
        return events


class RendezvousDetector:
    def __init__(self, threshold_m: float = 150.0, horizon: int = 4):
        self.threshold = threshold_m; self.horizon = horizon

    def detect(self, tracks: List[BernoulliTrack], n_mc: int = 100) -> List[Dict]:
        events = []
        n = len(tracks)
        for i in range(n):
            for j in range(i+1, n):
                ti, tj = tracks[i], tracks[j]
                sep = float(np.linalg.norm(ti.pos - tj.pos))
                if sep > self.threshold*8: continue
                sub = min(n_mc, ti.pf.n)
                idi = np.random.choice(ti.pf.n, sub, p=ti.pf.w/ti.pf.w.sum())
                idj = np.random.choice(tj.pf.n, sub, p=tj.pf.w/tj.pf.w.sum())
                xi = ti.pf.X[idi].copy(); xj = tj.pf.X[idj].copy()
                for _ in range(self.horizon):
                    ai = _MOU_ALPHA[np.random.choice(N_MODELS, sub, p=ti.pf.mu)]
                    aj = _MOU_ALPHA[np.random.choice(N_MODELS, sub, p=tj.pf.mu)]
                    si = _MOU_SIG_V[np.random.choice(N_MODELS, sub, p=ti.pf.mu)]
                    sj = _MOU_SIG_V[np.random.choice(N_MODELS, sub, p=tj.pf.mu)]
                    vi_new = ai[:,None]*xi[:,2:4] + si[:,None]*np.random.randn(sub,2)
                    vj_new = aj[:,None]*xj[:,2:4] + sj[:,None]*np.random.randn(sub,2)
                    xi = np.column_stack([xi[:,:2]+_DT*(xi[:,2:4]+vi_new)/2, vi_new])
                    xj = np.column_stack([xj[:,:2]+_DT*(xj[:,2:4]+vj_new)/2, vj_new])
                dists = np.linalg.norm(xi[:,:2]-xj[:,:2], axis=1)
                p_rv = float((dists<self.threshold).mean())
                if p_rv > 0.15:
                    ml = (xi[:,:2].mean(0)+xj[:,:2].mean(0))/2
                    events.append({'track_a':ti.tid,'track_b':tj.tid,'p_rendezvous':round(p_rv,4),
                                   'current_sep_m':round(sep,1),'horizon_scans':self.horizon,
                                   'predicted_location':ml.tolist(),
                                   'priority':'HIGH' if p_rv>0.5 else 'MEDIUM'})
        events.sort(key=lambda e: -e['p_rendezvous'])
        return events


class RoutePredictor:
    def forecast(self, track: BernoulliTrack, horizon: int = 8, n_mc: int = 120,
                 dt_per_scan: float = 60.0) -> List[Dict]:
        if track.pf.X is None: return []
        idx = np.random.choice(track.pf.n, n_mc, p=track.pf.w/track.pf.w.sum())
        pts = track.pf.X[idx].copy()
        waypoints = []
        for h in range(1, horizon+1):
            mi = np.random.choice(N_MODELS, n_mc, p=track.pf.mu)
            alpha_h = _MOU_ALPHA[mi]; sig_h = _MOU_SIG_V[mi]
            v_new = alpha_h[:,None]*pts[:,2:4] + sig_h[:,None]*np.random.randn(n_mc,2)
            pts = np.column_stack([pts[:,:2]+_DT*(pts[:,2:4]+v_new)/2, v_new])
            pol_blend = min(0.5, h/(horizon*2))
            if track.pol._fitted:
                pp, _ = track.pol.predict_location(track.last_seen+h*dt_per_scan, n_mc=40)
                pts[:,:2] += (pp - pts[:,:2])*pol_blend
            mean_pos = pts[:,:2].mean(0)
            unc = float(np.std(np.linalg.norm(pts[:,:2]-mean_pos, axis=1)))
            waypoints.append({'step':h,'time_ahead_s':round(h*dt_per_scan,0),
                               'position':mean_pos.tolist(),'uncertainty_m':round(unc,1),
                               'confidence':round(max(0.05,1.0-h*0.08),3)})
        return waypoints


class DynamicNetworkAnalyser:
    def __init__(self, coloc_dist: float = 350.0):
        self.coloc_dist = coloc_dist
        self._adj_accum: Dict[str, Dict[str,float]] = defaultdict(lambda: defaultdict(float))
        self._bc_history: Dict[str, List[float]] = defaultdict(list)
        self._all_tids: set = set()

    def analyse(self, tracks: List[BernoulliTrack], timestamp: float) -> List[Dict]:
        if len(tracks) < 2: return []
        tids = [t.tid for t in tracks]
        self._all_tids.update(tids)

        for t in tracks:
            for u in tracks:
                if t.tid >= u.tid: continue
                dist = float(np.linalg.norm(t.pos - u.pos))
                if dist < self.coloc_dist:
                    w = max(0.1, 1.0 - dist/self.coloc_dist)
                    self._adj_accum[t.tid][u.tid] += w
                    self._adj_accum[u.tid][t.tid] += w

        n = len(tracks)
        adj = np.zeros((n,n))
        for i, ta in enumerate(tracks):
            for j, tb in enumerate(tracks):
                if i == j: continue
                adj[i,j] = self._adj_accum[ta.tid].get(tb.tid, 0.0)

        bc_raw = _betweenness_centrality((adj > 0).astype(float))
        bc_weighted_deg = adj.sum(axis=1)

        for i, t in enumerate(tracks):
            self._bc_history[t.tid].append(float(bc_raw[i]))
            if len(self._bc_history[t.tid]) > 20: self._bc_history[t.tid].pop(0)

        clusters_idx: List[List[int]] = []
        visited = set()
        for i in range(n):
            if i in visited: continue
            nbrs = [j for j in range(n) if adj[i,j] > 0]
            if len(nbrs) >= 1:
                cluster = list(set([i] + nbrs))
                for c in cluster: visited.add(c)
                clusters_idx.append(cluster)

        results = []
        for ci in clusters_idx:
            members = [tracks[i] for i in ci]
            centre = np.mean([m.pos for m in members], axis=0)
            wd_vals = [float(bc_weighted_deg[i]) for i in ci]
            hub_tid = members[int(np.argmax(wd_vals))].tid
            recurring_tids = {t.tid for t in members
                              if sum(1 for v in self._bc_history.get(t.tid,[]) if v > 0) > 3}
            results.append({
                'cluster_id':    len(results),
                'member_ids':    [m.tid for m in members],
                'size':          len(members),
                'centre':        centre.tolist(),
                'hub_track':     hub_tid,
                'betweenness':   {tracks[i].tid: round(float(bc_raw[i]),4) for i in ci},
                'weighted_deg':  {tracks[i].tid: round(float(bc_weighted_deg[i]),2) for i in ci},
                'recurring':     len(recurring_tids) > 0,
                'significance':  'HIGH' if len(recurring_tids) > 0 else 'MEDIUM',
            })
        return results


class AnomalyEscalator:
    def __init__(self, window: int = 5, threshold: float = 0.72):
        self.window = window; self.threshold = threshold
        self.history: Dict[str,List[float]] = defaultdict(list)

    def update(self, tid: str, score: float) -> List:
        self.history[tid].append(score)
        hist = self.history[tid][-self.window:]
        alerts = []
        if score > self.threshold: alerts.append(('SPIKE', score))
        if len(hist) >= self.window and np.all(np.diff(hist) > 0): alerts.append(('ESCALATING', score))
        if len(hist) >= 3 and hist[-3]>0.5 and hist[-2]<0.3 and hist[-1]>0.6:
            alerts.append(('COUNTER_SURVEILLANCE', score))
        return alerts


class CredibilityFuser:
    RELIABILITY = {'GEOINT':0.90,'SIGINT':0.78,'COMMS':0.70,'HUMINT':0.62,'OSINT':0.48}

    def combine(self, evidence: List[Observation]) -> Dict:
        if not evidence: return {'belief':0.5,'plausibility':0.5,'conflict':0.0}
        mH=1.0; mnH=1.0; mT=1.0; K=0.0
        for obs in evidence[-8:]:
            r = self.RELIABILITY.get(obs.modality,0.5)*obs.confidence
            mh=r*0.85; mnh=(1-r)*0.10; mt=1-mh-mnh
            K = mH*mnh + mnH*mh; K = min(K,0.999)
            mH=(mH*mh+mH*mt+mT*mh)/(1-K)
            mnH=(mnH*mnh+mnH*mt+mT*mnh)/(1-K)
            mT=(mT*mt)/(1-K)
        return {'belief':round(float(np.clip(mH,0,1)),4),
                'plausibility':round(float(np.clip(mH+mT,0,1)),4),
                'conflict':round(float(np.clip(K,0,1)),4)}


class SensorScheduler:
    def schedule(self, tracks: List[BernoulliTrack], available_modalities: List[str]) -> List[Dict]:
        mod_w = np.array([MODALITY_WEIGHT.get(m,0.5) for m in available_modalities])
        recs = []
        for t in tracks[:8]:
            unc = max(t.pf.pos_uncertainty(), 1.0)
            scores = mod_w * t.r / unc
            bi = int(np.argmax(scores))
            recs.append({'track_id':t.tid,'recommended_modality':available_modalities[bi],
                         'expected_info_gain':round(float(scores[bi]),4),
                         'current_uncertainty_m':round(unc,1)})
        recs.sort(key=lambda x: -x['expected_info_gain'])
        return recs


class OperationalIntelligence:
    def analyse(self, tracks: List[BernoulliTrack], ts: float, dt_per_scan: float = 60.0) -> Dict:
        out = {'velocity_analysis':[], 'supply_predictions':[], 'geographic_flags':[],
               'model_transitions':[]}
        for t in tracks:
            if len(t._vel_history) > 5:
                speeds = [float(np.linalg.norm(v)) for v in t._vel_history[-10:]]
                mean_s = float(np.mean(speeds))
                trend  = float(np.polyfit(range(len(speeds)), speeds, 1)[0])
                out['velocity_analysis'].append({'track_id':t.tid,'mean_speed_mps':round(mean_s,2),
                                                  'speed_trend':round(trend,3),'model':t.pf.dominant_model})
                if mean_s < 0.5 and t.age > 10:
                    out['supply_predictions'].append({'track_id':t.tid,'type':'STATIONARY_DWELL',
                                                       'position':t.pos.tolist()})
            if len(t._pos_history) >= 2:
                rng = float(np.max(np.linalg.norm(
                    np.array(t._pos_history)-np.array(t._pos_history[0]), axis=1)))
                if rng > 3000:
                    out['geographic_flags'].append({'track_id':t.tid,'flag':'WIDE_AREA_MOVEMENT',
                                                    'range_m':round(rng,0)})
            if t.poss_mismatch > 0.4:
                out['model_transitions'].append({'track_id':t.tid,'mismatch':round(t.poss_mismatch,3),
                                                  'alert':'POSSIBLE_DECEPTION_OR_MODEL_ERROR'})
        return out


class ForwardBackwardSmoother:
    def __init__(self, lag: int = 6):
        self.lag = lag
        self._hist: Dict[str, List[Tuple[np.ndarray,np.ndarray]]] = defaultdict(list)

    def update(self, tid: str, X: np.ndarray, w: np.ndarray):
        self._hist[tid].append((X.copy(), w.copy()))
        if len(self._hist[tid]) > self.lag+2: self._hist[tid].pop(0)

    def smooth_pos(self, tid: str) -> Optional[np.ndarray]:
        hist = self._hist.get(tid,[])
        if len(hist) < 2: return None
        smoothed = [w @ X[:,:2] for X,w in hist[-self.lag:]]
        arr = np.array(smoothed)
        kernel = np.exp(-0.5*np.arange(len(smoothed))**2/2.0)[::-1]
        kernel /= kernel.sum()
        return (kernel[:,None]*arr).sum(0)



# ═══════════════════════ v6 EXTENSIONS ═══════════════════════

from abc import ABC, abstractmethod

# ─── Forward import from v5 base ─────────────────────────────────────────────
# We import everything from v5 and override/extend selectively
import sys, importlib

# ──────────────────────────────────────────────────────────────────────────────
#  DOMAIN PROFILE  (replaces all global magic numbers)
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class DomainProfile:
    """
    Fully parameterised domain bundle.  Pass to ARIAIntelEngine; every module
    reads from profile rather than module-level globals.  Swap profiles at
    construction time to reinterpret the same codebase for a different domain.
    """
    name: str

    # ── Scan / sensor ──────────────────────────────────────────────────────
    scan_dt_s: float           = 60.0     # seconds between scans
    pos_noise_m: float         = 5.0      # 1-sigma position measurement noise
    modality_weights: Dict     = field(default_factory=lambda: {
        'GEOINT':0.95,'SIGINT':0.82,'COMMS':0.75,'HUMINT':0.65,'OSINT':0.55})

    # ── PMBM filter ────────────────────────────────────────────────────────
    p_detection:   float       = 0.85
    p_survival:    float       = 0.995
    r_birth:       float       = 0.65
    r_confirm:     float       = 0.55
    r_prune:       float       = 0.05
    r_dormant:     float       = 0.04
    dormant_timeout: int       = 40
    gate_sigma:    float       = 3.7196   # chi2(0.999, 2) = 13.82 → sqrt ≈ 3.72

    # ── Motion models (MOU) ────────────────────────────────────────────────
    mou_models: Dict           = field(default_factory=lambda: {
        'foot':       {'theta':0.30, 'sigma':2.0},
        'vehicle':    {'theta':0.10, 'sigma':8.0},
        'stationary': {'theta':2.00, 'sigma':0.5},
        'fast':       {'theta':0.05, 'sigma':15.0},
    })
    model_trans: np.ndarray    = field(default_factory=lambda: np.array([
        [0.85,0.10,0.04,0.01],
        [0.05,0.88,0.02,0.05],
        [0.15,0.05,0.78,0.02],
        [0.02,0.20,0.01,0.77],
    ]))

    # ── Rendezvous (stacked warner) ────────────────────────────────────────
    rv_threshold_m:  float     = 150.0    # meeting distance
    rv_horizon_scans: int      = 4        # MOU MC horizon (short)
    rv_warning_horizon_s: float= 1800.0   # 30-minute CPA warning window
    rv_sep_rate_window: int    = 8        # scans for closure-rate fit
    rv_pol_window_s:   float   = 3600.0   # PoL cross-prediction lookahead

    # ── Tradecraft detection ────────────────────────────────────────────────
    brush_pass_m:     float    = 60.0
    parallel_route_m: float    = 80.0     # lateral offset for mobile tail
    parallel_vel_cos: float    = 0.97     # cosine similarity for heading match
    parallel_scans:   int      = 6        # consecutive scans to confirm tail
    mode_trans_m:     float    = 50.0     # vehicle stop → foot appear radius
    mode_trans_scans: int      = 2        # scan window for mode transition
    loiter_mult:      float    = 3.0      # dwell = loiter_mult × PoL baseline
    loiter_min_s:     float    = 300.0    # min dwell to flag loiter
    cover_stop_m:     float    = 300.0    # max dist from PoL location → cover stop
    cover_stop_hvl_m: float    = 800.0    # cover stop within X m of HVL
    chokepoint_m:     float    = 40.0     # same-cell repeat passage radius
    chokepoint_n:     int      = 3        # repeat passes to flag
    dead_drop_spread: Tuple    = (60.0, 1800.0)

    # ── Network analysis ───────────────────────────────────────────────────
    coloc_dist_m:    float     = 350.0
    courier_speed_thresh: float= 3.0      # m/s minimum speed for courier
    courier_contact_n:    int  = 3        # unique contacts to flag courier
    handler_contact_max:  int  = 2        # handlers have few contacts
    handler_stable_scans: int  = 10       # handlers are stable

    # ── Threat scoring ─────────────────────────────────────────────────────
    threat_weights: np.ndarray = field(default_factory=lambda:
        np.array([0.23,0.18,0.13,0.13,0.08,0.08,0.10,0.07]))
    hvl_radius_m:  float       = 600.0


# ── Four domain presets ────────────────────────────────────────────────────────

def UrbanHUMINT() -> DomainProfile:
    return DomainProfile(name='UrbanHUMINT')  # all defaults

def Maritime() -> DomainProfile:
    return DomainProfile(
        name='Maritime',
        scan_dt_s=3600.0,           # 1-hour satellite/AIS refresh
        pos_noise_m=200.0,          # AIS position accuracy
        p_detection=0.75,           # ships can go dark (AIS off)
        rv_threshold_m=2000.0,      # ship-to-ship meeting range
        rv_warning_horizon_s=7200.0,# 2-hour CPA window
        brush_pass_m=500.0,
        parallel_route_m=800.0,
        parallel_vel_cos=0.99,
        mode_trans_m=500.0,
        coloc_dist_m=3000.0,
        hvl_radius_m=5000.0,
        courier_speed_thresh=2.0,   # knots equivalent
        mou_models={
            'drifting':  {'theta':0.50, 'sigma':0.5},
            'transiting':{'theta':0.05, 'sigma':3.0},
            'anchored':  {'theta':5.00, 'sigma':0.1},
            'fast_craft':{'theta':0.02, 'sigma':8.0},
        },
        model_trans=np.array([
            [0.80,0.15,0.04,0.01],
            [0.05,0.88,0.05,0.02],
            [0.20,0.05,0.74,0.01],
            [0.02,0.30,0.01,0.67],
        ]),
    )

def Airspace() -> DomainProfile:
    return DomainProfile(
        name='Airspace',
        scan_dt_s=5.0,              # radar sweep
        pos_noise_m=50.0,
        p_detection=0.98,
        rv_threshold_m=1000.0,
        rv_warning_horizon_s=600.0, # 10-min CPA for aircraft
        brush_pass_m=300.0,
        coloc_dist_m=2000.0,
        hvl_radius_m=20000.0,
        parallel_route_m=500.0,
        parallel_vel_cos=0.995,
        mou_models={
            'hovering':  {'theta':1.00, 'sigma':3.0},
            'fixed_wing':{'theta':0.02, 'sigma':20.0},
            'gliding':   {'theta':0.10, 'sigma':5.0},
            'fast_jet':  {'theta':0.01, 'sigma':50.0},
        },
        model_trans=np.array([
            [0.90,0.05,0.04,0.01],
            [0.02,0.92,0.03,0.03],
            [0.05,0.10,0.83,0.02],
            [0.01,0.15,0.01,0.83],
        ]),
    )

def VehicleConvoy() -> DomainProfile:
    return DomainProfile(
        name='VehicleConvoy',
        scan_dt_s=10.0,
        pos_noise_m=3.0,            # GPS-quality
        p_detection=0.92,
        rv_threshold_m=30.0,        # vehicles touch at ~10m
        rv_warning_horizon_s=300.0, # 5-min CPA
        brush_pass_m=20.0,
        parallel_route_m=15.0,
        parallel_vel_cos=0.99,
        coloc_dist_m=100.0,
        hvl_radius_m=500.0,
        mou_models={
            'stopped':   {'theta':5.00, 'sigma':0.2},
            'slow_roll': {'theta':0.50, 'sigma':2.0},
            'highway':   {'theta':0.05, 'sigma':8.0},
            'sprint':    {'theta':0.02, 'sigma':15.0},
        },
        model_trans=np.array([
            [0.75,0.20,0.04,0.01],
            [0.15,0.72,0.08,0.05],
            [0.03,0.10,0.82,0.05],
            [0.02,0.08,0.10,0.80],
        ]),
    )


# ──────────────────────────────────────────────────────────────────────────────
#  DETECTOR INTERFACE  (composable plugin architecture)
# ──────────────────────────────────────────────────────────────────────────────

class BaseDetector(ABC):
    """
    All detectors implement this interface.
    Engine calls detect(tracks, context) → List[Alert].
    context carries: timestamp, scan_index, hvls, profile, engine_ref
    """
    def __init__(self, profile: DomainProfile):
        self.profile = profile

    @abstractmethod
    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        ...

    @property
    @abstractmethod
    def name(self) -> str:
        ...


# ──────────────────────────────────────────────────────────────────────────────
#  RENDEZVOUS WARNING  — three stacked methods
# ──────────────────────────────────────────────────────────────────────────────

class ExtendedRendezvousWarner(BaseDetector):
    """
    Stacks three complementary CPA prediction methods.
    Fires on whichever gives the longest valid warning time.

    Method 1 — Geometric Velocity Intercept
      Fit least-squares line through last N position history points for each
      track.  Compute analytical line-line closest-approach.  Warning time =
      current_sep / approach_rate, capped at rv_warning_horizon_s.

    Method 2 — Separation Rate Extrapolation
      Compute d(separation)/dt over sep_rate_window scans.  Fit linear trend.
      Project to when separation ≤ threshold.  Robust even when heading noisy.

    Method 3 — PoL Cross-Prediction
      If both tracks have fitted PoL models, sample PoL-predicted positions at
      t_now + k*scan_dt for k = 1…horizon.  If predicted positions converge
      within threshold, fire with time-to-meeting.
    """
    name = 'ExtendedRendezvousWarner'

    def __init__(self, profile: DomainProfile):
        super().__init__(profile)
        self._sep_history: Dict[Tuple, List[Tuple[float,float]]] = defaultdict(list)
        self._mou_alpha: Optional[np.ndarray] = None
        self._mou_sigv:  Optional[np.ndarray] = None
        self._vel_fit_cache: Dict = {}

    def _build_mou_tables(self, profile: DomainProfile):
        keys = list(profile.mou_models.keys())
        dt = profile.scan_dt_s
        self._mou_alpha = np.array([
            np.exp(-profile.mou_models[k]['theta'] * dt) for k in keys])
        self._mou_sigv  = np.array([
            profile.mou_models[k]['sigma'] *
            np.sqrt((1 - np.exp(-2*profile.mou_models[k]['theta']*dt)) /
                    (2*profile.mou_models[k]['theta']))
            for k in keys])
        self._n_models  = len(keys)

    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        if self._mou_alpha is None:
            self._build_mou_tables(self.profile)

        p      = self.profile
        ts     = context['timestamp']
        scan   = context.get('scan_index', 0)
        events = []
        n      = len(tracks)
        run_pol = (scan % 5 == 0)   # PoL cross-predict throttled to every 5 scans

        for i in range(n):
            for j in range(i+1, n):
                ti, tj = tracks[i], tracks[j]
                sep = float(np.linalg.norm(ti.pos - tj.pos))
                key = (ti.tid, tj.tid)
                self._sep_history[key].append((ts, sep))
                if len(self._sep_history[key]) > 30:
                    self._sep_history[key].pop(0)

                best_warning = None

                # ── Method 1: Geometric velocity intercept ────────────────
                if len(ti._pos_history) >= 4 and len(tj._pos_history) >= 4:
                    w1 = self._geometric_intercept(ti, tj, ts, p)
                    if w1 and w1['eta_s'] <= p.rv_warning_horizon_s:
                        if best_warning is None or w1['eta_s'] < best_warning['eta_s']:
                            best_warning = w1

                # ── Method 2: Separation rate extrapolation ───────────────
                hist = self._sep_history[key]
                if len(hist) >= max(3, p.rv_sep_rate_window // 2):
                    w2 = self._sep_rate_extrap(hist, ts, p)
                    if w2 and w2['eta_s'] <= p.rv_warning_horizon_s:
                        if best_warning is None or w2['eta_s'] < best_warning['eta_s']:
                            best_warning = w2

                # ── Method 3: PoL cross-prediction (throttled) ───────────
                if run_pol and ti.pol._fitted and tj.pol._fitted:
                    w3 = self._pol_cross_predict(ti, tj, ts, p)
                    if w3 and w3['eta_s'] <= p.rv_warning_horizon_s:
                        if best_warning is None or w3['eta_s'] < best_warning['eta_s']:
                            best_warning = w3

                if best_warning:
                    eta_min = best_warning['eta_s'] / 60.0
                    priority = ('IMMEDIATE' if eta_min < 5 else
                                'HIGH'      if eta_min < 15 else
                                'MEDIUM'    if eta_min < 30 else 'LOW')
                    events.append({
                        'type':        'RENDEZVOUS_WARNING',
                        'track_a':     ti.tid,
                        'track_b':     tj.tid,
                        'current_sep_m': round(sep, 1),
                        'eta_s':       round(best_warning['eta_s'], 0),
                        'eta_min':     round(eta_min, 1),
                        'method':      best_warning['method'],
                        'confidence':  round(best_warning.get('confidence', 0.5), 3),
                        'predicted_location': best_warning.get('location', None),
                        'priority':    priority,
                        'timestamp':   ts,
                    })

        events.sort(key=lambda e: e['eta_s'])
        return events

    def _fit_vel_cached(self, t) -> np.ndarray:
        """Least-squares velocity from position history with LRU-style cache."""
        ph = list(t._pos_history)
        cache_key = (t.tid, len(ph))
        if cache_key in getattr(self, '_vel_fit_cache', {}):
            return self._vel_fit_cache[cache_key]
        if not hasattr(self, '_vel_fit_cache') or self._vel_fit_cache is None:
            self._vel_fit_cache = {}
        ph_arr = np.array(ph[-8:], dtype=float)
        if len(ph_arr) < 3:
            result = t.vel.copy()
        else:
            idx = np.arange(len(ph_arr), dtype=float)
            vx = float(np.dot(idx - idx.mean(), ph_arr[:,0] - ph_arr[:,0].mean()) /
                        max(float(np.dot(idx - idx.mean(), idx - idx.mean())), 1e-9))
            vy = float(np.dot(idx - idx.mean(), ph_arr[:,1] - ph_arr[:,1].mean()) /
                        max(float(np.dot(idx - idx.mean(), idx - idx.mean())), 1e-9))
            result = np.array([vx, vy])
        # Bound cache size
        if len(self._vel_fit_cache) > 200:
            oldest = next(iter(self._vel_fit_cache))
            del self._vel_fit_cache[oldest]
        self._vel_fit_cache[cache_key] = result
        return result

    def _geometric_intercept(self, ti, tj, ts, p) -> Optional[Dict]:
        """
        Fit velocity vector to each track's recent position history.
        Project as parametric rays.  Find time of closest approach (CPA).
        """
        ph_i = list(ti._pos_history)
        ph_j = list(tj._pos_history)
        if len(ph_i) < 3 or len(ph_j) < 3:
            return None

        vi = self._fit_vel_cached(ti)
        vj = self._fit_vel_cached(tj)
        pi_ = ti.pos.copy()
        pj_ = tj.pos.copy()
        dv  = vi - vj
        dp  = pi_ - pj_
        dv2 = float(np.dot(dv, dv))
        if dv2 < 1e-6:
            return None  # parallel tracks, not converging

        t_cpa = -float(np.dot(dp, dv)) / dv2  # scans to CPA
        if t_cpa <= 0:
            return None  # diverging

        eta_s = t_cpa * p.scan_dt_s
        pi_cpa = pi_ + vi * t_cpa
        pj_cpa = pj_ + vj * t_cpa
        cpa_sep = float(np.linalg.norm(pi_cpa - pj_cpa))
        if cpa_sep > p.rv_threshold_m * 2:
            return None  # won't actually meet

        loc = ((pi_cpa + pj_cpa) / 2).tolist()
        # Confidence: lower when CPA sep is large relative to threshold
        conf = float(np.clip(1.0 - cpa_sep / p.rv_threshold_m, 0.1, 1.0))
        return {'method': 'GEOMETRIC_INTERCEPT', 'eta_s': eta_s,
                'location': loc, 'confidence': conf, 'cpa_sep_m': round(cpa_sep, 1)}

    def _sep_rate_extrap(self, hist: List[Tuple], ts: float, p) -> Optional[Dict]:
        """
        Fit linear trend to recent separations.  Project to threshold.
        """
        seps = np.array([s for _, s in hist[-p.rv_sep_rate_window:]])
        times = np.array([t for t, _ in hist[-p.rv_sep_rate_window:]])
        if len(seps) < 3:
            return None
        # Linear fit: sep = a*t + b
        t_norm = (times - times[0]) / p.scan_dt_s
        coeffs = np.polyfit(t_norm, seps, 1)
        slope  = float(coeffs[0])  # Δm per scan
        if slope >= 0:
            return None  # separating

        current_sep = float(seps[-1])
        # scans to reach rv_threshold_m from current position
        scans_to_rv = (current_sep - p.rv_threshold_m) / (-slope)
        if scans_to_rv <= 0:
            scans_to_rv = 1.0
        eta_s = scans_to_rv * p.scan_dt_s
        # Confidence: R² of linear fit
        predicted = np.polyval(coeffs, t_norm)
        ss_res = float(np.sum((seps - predicted)**2))
        ss_tot = float(np.sum((seps - seps.mean())**2))
        r2 = float(1 - ss_res / (ss_tot + 1e-9))
        conf = float(np.clip(r2, 0.05, 0.99))
        return {'method': 'SEP_RATE_EXTRAP', 'eta_s': eta_s,
                'confidence': conf, 'slope_m_per_scan': round(slope, 2)}

    def _pol_cross_predict(self, ti, tj, ts: float, p) -> Optional[Dict]:
        """
        Vectorised PoL cross-prediction: sample all horizon steps in one batch.
        Only called when both tracks have fitted PoL models.
        """
        dt            = p.scan_dt_s
        horizon_steps = min(int(p.rv_pol_window_s / dt), 20)  # hard cap: 20 min warning
        if horizon_steps < 1:
            return None

        # Batch predict: evaluate PoL at all future timestamps simultaneously
        t_futures = np.array([ts + k * dt for k in range(1, horizon_steps + 1)])
        n_mc_pol  = 8   # minimal MC for PoL cross-predict

        pi_preds = []; pj_preds = []; ui_list = []; uj_list = []
        for t_fut in t_futures:
            pi, ui = ti.pol.predict_location(t_fut, n_mc=n_mc_pol)
            pj, uj = tj.pol.predict_location(t_fut, n_mc=n_mc_pol)
            pi_preds.append(pi); pj_preds.append(pj)
            ui_list.append(ui); uj_list.append(uj)

        pi_arr = np.array(pi_preds); pj_arr = np.array(pj_preds)
        seps   = np.linalg.norm(pi_arr - pj_arr, axis=1)
        meets  = np.where(seps < p.rv_threshold_m)[0]
        if len(meets) == 0:
            return None

        k_best = int(meets[0])
        eta_s  = float((k_best + 1) * dt)
        total_unc = float(ui_list[k_best] + uj_list[k_best])
        conf   = float(np.clip(1.0 - total_unc / (p.rv_threshold_m * 4), 0.1, 0.95))
        loc    = ((pi_arr[k_best] + pj_arr[k_best]) / 2).tolist()
        return {'method': 'POL_CROSS_PREDICT', 'eta_s': eta_s,
                'location': loc, 'confidence': conf}


# ──────────────────────────────────────────────────────────────────────────────
#  PARALLEL ROUTE SURVEILLANCE DETECTOR
# ──────────────────────────────────────────────────────────────────────────────

class ParallelRouteSurveillanceDetector(BaseDetector):
    """
    Detects mobile surveillance: track B follows track A at consistent lateral
    offset with matching heading for N consecutive scans.

    Criteria:
      • Cosine similarity of velocity vectors ≥ parallel_vel_cos
      • Lateral offset (perpendicular to A's heading) ≤ parallel_route_m
      • Longitudinal separation ≤ 3× parallel_route_m (close enough to tail)
      • Persists for parallel_scans consecutive scans
    """
    name = 'ParallelRouteSurveillance'

    def __init__(self, profile: DomainProfile):
        super().__init__(profile)
        self._parallel_counts: Dict[Tuple[str,str], int] = defaultdict(int)

    @staticmethod
    def _history_vel(t) -> np.ndarray:
        """Least-squares velocity from position history — more stable than MOU particle mean."""
        ph = np.array(list(t._pos_history)[-8:], dtype=float)
        if len(ph) < 3:
            return t.vel
        idx = np.arange(len(ph), dtype=float)
        vx = float(np.polyfit(idx, ph[:,0], 1)[0])
        vy = float(np.polyfit(idx, ph[:,1], 1)[0])
        return np.array([vx, vy])

    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        p      = self.profile
        events = []
        n      = len(tracks)

        for i in range(n):
            for j in range(i+1, n):
                ta, tb = tracks[i], tracks[j]
                va = self._history_vel(ta)
                vb = self._history_vel(tb)
                va_norm = float(np.linalg.norm(va))
                vb_norm = float(np.linalg.norm(vb))
                if va_norm < 0.5 or vb_norm < 0.5:
                    continue

                cos_sim = float(np.dot(va, vb) / (va_norm * vb_norm))
                if cos_sim < p.parallel_vel_cos:
                    self._parallel_counts[(ta.tid, tb.tid)] = 0
                    continue

                # Project separation onto perpendicular of A's heading
                hat_a = va / va_norm
                perp_a = np.array([-hat_a[1], hat_a[0]])
                dp = tb.pos - ta.pos
                lateral  = abs(float(np.dot(dp, perp_a)))
                longit   = abs(float(np.dot(dp, hat_a)))

                if lateral > p.parallel_route_m or longit > p.parallel_route_m * 3:
                    self._parallel_counts[(ta.tid, tb.tid)] = 0
                    continue

                key = (ta.tid, tb.tid)
                self._parallel_counts[key] += 1

                if self._parallel_counts[key] >= p.parallel_scans:
                    events.append({
                        'type':          'PARALLEL_SURVEILLANCE',
                        'subject':       ta.tid,
                        'surveillant':   tb.tid,
                        'lateral_m':     round(lateral, 1),
                        'longitudinal_m':round(longit, 1),
                        'heading_cos':   round(cos_sim, 4),
                        'consecutive_scans': self._parallel_counts[key],
                        'timestamp':     context['timestamp'],
                        'severity':      'HIGH',
                    })

        return events


# ──────────────────────────────────────────────────────────────────────────────
#  MODE TRANSITION DETECTOR  (vehicle ↔ foot handoff)
# ──────────────────────────────────────────────────────────────────────────────

class ModeTransitionDetector(BaseDetector):
    """
    Detects vehicle-to-foot or foot-to-vehicle mode transitions indicating:
      • Driver handoff / asset swap
      • Cache exchange at vehicle stop
      • Cover-vehicle dismount

    Trigger: track A (vehicle model, speed > threshold) decelerates to zero
    OR disappears, AND track B (foot model, new) appears within mode_trans_m
    within mode_trans_scans scans.

    Uses MOU dominant_model classification and velocity magnitude.
    """
    name = 'ModeTransition'

    def __init__(self, profile: DomainProfile):
        super().__init__(profile)
        self._vehicle_stops: List[Dict] = []     # {tid, pos, ts, scan}
        self._mou_keys = list(profile.mou_models.keys())
        # Identify which model indices are "vehicle-class" vs "foot-class"
        # Heuristic: sigma > 3 = vehicle-class; sigma ≤ 3 = foot/slow class
        self._vehicle_models = {k for k,v in profile.mou_models.items()
                                if v['sigma'] > 3.0}
        self._foot_models    = {k for k,v in profile.mou_models.items()
                                if v['sigma'] <= 3.0}

    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        p       = self.profile
        ts      = context['timestamp']
        scan    = context['scan_index']
        events  = []

        # Record current vehicle-class tracks that are nearly stopped
        new_stops = []
        for t in tracks:
            speed = float(np.linalg.norm(t.vel))
            dm    = t.pf.dominant_model
            if dm in self._vehicle_models and speed < 2.5:
                new_stops.append({'tid': t.tid, 'pos': t.pos.copy(),
                                   'ts': ts, 'scan': scan})

        # Check: did a foot-class track appear near a recent vehicle stop?
        to_remove = []
        for stop in self._vehicle_stops:
            if scan - stop['scan'] > p.mode_trans_scans:
                to_remove.append(stop)
                continue
            for t in tracks:
                dm = t.pf.dominant_model
                if dm not in self._foot_models:
                    continue
                if t.age > p.mode_trans_scans + 2:
                    continue  # not newly appeared
                dist = float(np.linalg.norm(t.pos - stop['pos']))
                if dist <= p.mode_trans_m:
                    events.append({
                        'type':          'MODE_TRANSITION',
                        'vehicle_track': stop['tid'],
                        'foot_track':    t.tid,
                        'stop_pos':      stop['pos'].tolist(),
                        'foot_pos':      t.pos.tolist(),
                        'dist_m':        round(dist, 1),
                        'delay_scans':   scan - stop['scan'],
                        'timestamp':     ts,
                        'severity':      'HIGH',
                        'interpretation': 'VEHICLE_HANDOFF_OR_DISMOUNT',
                    })
                    to_remove.append(stop)
                    break

        for s in to_remove:
            if s in self._vehicle_stops:
                self._vehicle_stops.remove(s)

        self._vehicle_stops.extend(new_stops)
        # Prune old stops
        self._vehicle_stops = [s for s in self._vehicle_stops
                                if scan - s['scan'] <= p.mode_trans_scans + 1]
        return events


# ──────────────────────────────────────────────────────────────────────────────
#  LOITER ANOMALY DETECTOR
# ──────────────────────────────────────────────────────────────────────────────

class LoiterAnomalyDetector(BaseDetector):
    """
    Flags when a track's dwell time at a non-PoL location exceeds
    loiter_mult × PoL-baseline dwell.

    Dwell detection: track stays within loiter_min_s * speed_threshold of
    a fixed point across consecutive scans.  Timer starts on entry.

    PoL baseline dwell: estimated from PoL GMM — how long does the track
    typically stay in any one cluster?  Approximated as inverse of the
    transition rate between GMM components (empirical from visit history).
    """
    name = 'LoiterAnomaly'

    def __init__(self, profile: DomainProfile):
        super().__init__(profile)
        self._dwell_start: Dict[str, Tuple[float, np.ndarray]] = {}
        self._dwell_anchor: Dict[str, np.ndarray] = {}

    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        p      = self.profile
        ts     = context['timestamp']
        events = []
        dwell_radius = max(p.pos_noise_m * 4, 30.0)

        for t in tracks:
            tid = t.tid
            if tid not in self._dwell_anchor:
                self._dwell_anchor[tid] = t.pos.copy()
                self._dwell_start[tid]  = (ts, t.pos.copy())
                continue

            dist_from_anchor = float(np.linalg.norm(t.pos - self._dwell_anchor[tid]))
            if dist_from_anchor > dwell_radius:
                # Track has moved — reset
                self._dwell_anchor[tid] = t.pos.copy()
                self._dwell_start[tid]  = (ts, t.pos.copy())
                continue

            # Still dwelling
            start_ts, anchor_pos = self._dwell_start[tid]
            dwell_s = ts - start_ts
            if dwell_s < p.loiter_min_s:
                continue

            # Check if location is anomalous (not in PoL)
            if t.pol._fitted:
                anom = t.pol.anomaly_score(ts, anchor_pos)
                # anom > 0.65 means low PoL probability → unexpected location
                if anom > 0.65:
                    # Estimate baseline dwell from PoL history
                    baseline_dwell = self._pol_baseline_dwell(t.pol, p)
                    if dwell_s > baseline_dwell * p.loiter_mult:
                        events.append({
                            'type':          'LOITER_ANOMALY',
                            'track':         tid,
                            'position':      anchor_pos.tolist(),
                            'dwell_s':       round(dwell_s, 0),
                            'dwell_min':     round(dwell_s/60, 1),
                            'baseline_dwell_s': round(baseline_dwell, 0),
                            'loiter_ratio':  round(dwell_s/max(baseline_dwell,1), 2),
                            'pol_anomaly':   round(anom, 3),
                            'timestamp':     ts,
                            'severity':      'HIGH' if dwell_s > baseline_dwell*5 else 'MEDIUM',
                        })
            else:
                # No PoL fitted — flag on absolute duration only
                if dwell_s > p.loiter_min_s * 4:
                    events.append({
                        'type':        'LOITER_ANOMALY',
                        'track':       tid,
                        'position':    anchor_pos.tolist(),
                        'dwell_s':     round(dwell_s, 0),
                        'dwell_min':   round(dwell_s/60, 1),
                        'baseline_dwell_s': None,
                        'loiter_ratio': None,
                        'pol_anomaly':  None,
                        'timestamp':    ts,
                        'severity':    'LOW',
                    })

        return events

    @staticmethod
    def _pol_baseline_dwell(pol, p: DomainProfile) -> float:
        """
        Estimate typical dwell from PoL visit history.
        Baseline = median inter-visit duration / n_clusters.
        """
        if not pol.obs or len(pol.obs) < 4:
            return p.loiter_min_s * 2
        times = sorted(v[0] for v in pol.obs)
        gaps  = np.diff(times)
        if len(gaps) == 0:
            return p.loiter_min_s * 2
        return float(np.median(gaps))


# ──────────────────────────────────────────────────────────────────────────────
#  COVER STOP DETECTOR
# ──────────────────────────────────────────────────────────────────────────────

class CoverStopDetector(BaseDetector):
    """
    Detects the classic cover-stop tradecraft: target visits a PoL-consistent
    innocuous location, but the timing correlates with proximity to an HVL.

    Detection logic:
      1. Track visits a PoL-consistent location (pol_anomaly < 0.45 → "routine")
      2. The visit is within cover_stop_hvl_m of an HVL
      3. The timing is clustered (multiple such visits within hvl_radius)
         OR this visit immediately preceded/followed HVL proximity

    Operationally: "He stops at the same cafe every time before going near the
    embassy.  The cafe is his cover stop."
    """
    name = 'CoverStop'

    def __init__(self, profile: DomainProfile):
        super().__init__(profile)
        self._cover_history: Dict[str, List[Dict]] = defaultdict(list)

    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        p      = self.profile
        ts     = context['timestamp']
        hvls   = context.get('hvls', [])
        if not hvls:
            return []

        events = []
        for t in tracks:
            if not t.pol._fitted:
                continue

            pol_anom = t.pol.anomaly_score(ts, t.pos)
            # Cover stop: location is "routine enough" — not extreme anomaly
            # High threshold because target may visit cover stop more than home
            if pol_anom > 0.80:
                continue

            # PoL-consistent: is it near an HVL?
            for hvl in hvls:
                dist_hvl = float(np.linalg.norm(t.pos - hvl))
                if dist_hvl > p.cover_stop_hvl_m:
                    continue

                rec = {'tid': t.tid, 'pos': t.pos.copy(), 'ts': ts,
                       'pol_anom': pol_anom, 'hvl_dist': dist_hvl,
                       'hvl': hvl.tolist()}
                self._cover_history[t.tid].append(rec)
                if len(self._cover_history[t.tid]) > 20:
                    self._cover_history[t.tid].pop(0)

                hist = self._cover_history[t.tid]
                if len(hist) >= 2:
                    # Check repeated cover-stop pattern near same HVL
                    same_cell = [h for h in hist
                                 if np.linalg.norm(h['pos'] - t.pos) < p.cover_stop_m
                                 and h['ts'] != ts]
                    if len(same_cell) >= 1:
                        events.append({
                            'type':       'COVER_STOP',
                            'track':      t.tid,
                            'position':   t.pos.tolist(),
                            'hvl':        hvl.tolist(),
                            'hvl_dist_m': round(dist_hvl, 1),
                            'pol_anomaly': round(pol_anom, 3),
                            'repeat_count': len(same_cell) + 1,
                            'timestamp':  ts,
                            'severity':   'IMMEDIATE' if len(same_cell) >= 2 else 'HIGH',
                            'interpretation': 'ROUTINE_STOP_NEAR_HVL',
                        })

        return events


# ──────────────────────────────────────────────────────────────────────────────
#  CHOKEPOINT SURVEILLANCE DETECTOR
# ──────────────────────────────────────────────────────────────────────────────

class ChokepointSurveillanceDetector(BaseDetector):
    """
    Detects a track repeatedly passing through the same geographic chokepoint
    facing different headings — indicates static surveillance of who comes
    through (door, alley, bridge, gate).

    Trigger:
      • Same cell (within chokepoint_m) visited ≥ chokepoint_n times
      • Heading variance across visits is HIGH (bidirectional passage)
      • Visits are spread across multiple time windows (not just passing through)
    """
    name = 'ChokepointSurveillance'

    def __init__(self, profile: DomainProfile):
        super().__init__(profile)
        self._cell_visits: Dict[str, Dict[str, List[Dict]]] = defaultdict(
            lambda: defaultdict(list))   # tid → cell_key → [visits]

    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        p      = self.profile
        ts     = context['timestamp']
        events = []

        for t in tracks:
            speed = float(np.linalg.norm(t.vel))
            if speed < 0.3:
                continue  # stationary — not passing through

            cell_key = (int(t.pos[0] / p.chokepoint_m),
                        int(t.pos[1] / p.chokepoint_m))
            cell_str = f"{cell_key[0]}_{cell_key[1]}"

            heading_rad = float(np.arctan2(t.vel[1], t.vel[0]))
            self._cell_visits[t.tid][cell_str].append({
                'ts': ts, 'pos': t.pos.copy(), 'heading': heading_rad, 'speed': speed
            })
            # Prune to last 50 visits
            self._cell_visits[t.tid][cell_str] = \
                self._cell_visits[t.tid][cell_str][-50:]

            visits = self._cell_visits[t.tid][cell_str]
            if len(visits) < p.chokepoint_n:
                continue

            # Heading variance: use circular variance
            headings = np.array([v['heading'] for v in visits])
            c_var = 1.0 - float(np.abs(np.mean(np.exp(1j * headings))))
            # c_var close to 1.0 = high variance (bidirectional)
            if c_var < 0.3:
                continue  # all going same direction — just a route

            # Time spread: visits should be spread across >2 distinct sessions
            times = sorted(v['ts'] for v in visits)
            gaps  = np.diff(times)
            n_sessions = int(np.sum(gaps > p.scan_dt_s * 10)) + 1

            if n_sessions >= 2:
                events.append({
                    'type':          'CHOKEPOINT_SURVEILLANCE',
                    'track':         t.tid,
                    'chokepoint':    t.pos.tolist(),
                    'cell':          cell_str,
                    'visit_count':   len(visits),
                    'heading_var':   round(c_var, 3),
                    'n_sessions':    n_sessions,
                    'timestamp':     ts,
                    'severity':      'HIGH',
                    'interpretation': 'SURVEILLANCE_OF_CHOKEPOINT',
                })

        return events


# ──────────────────────────────────────────────────────────────────────────────
#  NETWORK ROLE INFERENCE  (Courier / Handler / Asset)
# ──────────────────────────────────────────────────────────────────────────────

class NetworkRoleInference(BaseDetector):
    """
    Classifies confirmed tracks into operational network roles using graph-theoretic
    features combined with motion model and PoL characteristics.

    Roles:
      COURIER  — high mobility, short meetings, many unique contacts across the
                 network.  High degree centrality, high avg speed, low PoL anomaly
                 (routine routes).

      HANDLER  — stable PoL, few contacts, receives couriers.  Low mobility,
                 high betweenness (gateway node), long contact durations.

      ASSET    — irregular activity (high PoL anomaly), meets only 1-2 tracks
                 (handler only), does not initiate meetings.  Low degree.

      UNKNOWN  — insufficient data or does not fit taxonomy.

    Features used per track:
      • avg_speed (from vel_history)
      • degree (n unique co-location events)
      • betweenness centrality (from DynamicNetworkAnalyser)
      • pol_anomaly (from score breakdown)
      • contact_diversity (unique tids met)
      • dwell_ratio (fraction of scans near same position)
    """
    name = 'NetworkRoleInference'

    def __init__(self, profile: DomainProfile):
        super().__init__(profile)
        self._contact_log: Dict[str, set] = defaultdict(set)
        self._meet_durations: Dict[str, List[float]] = defaultdict(list)
        self._role_history: Dict[str, List[str]] = defaultdict(list)

    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        p   = self.profile
        ts  = context['timestamp']
        bc  = context.get('betweenness', {})    # from network analyser

        # Update contact log from current co-locations
        n = len(tracks)
        for i in range(n):
            for j in range(i+1, n):
                ti, tj = tracks[i], tracks[j]
                sep = float(np.linalg.norm(ti.pos - tj.pos))
                if sep < p.coloc_dist_m:
                    self._contact_log[ti.tid].add(tj.tid)
                    self._contact_log[tj.tid].add(ti.tid)
                    self._meet_durations[f"{ti.tid}_{tj.tid}"].append(ts)

        # Batch relative classification
        batch_roles = self._classify_batch(tracks)

        events = []
        for t in tracks:
            if t.age < 10:
                continue

            avg_speed   = float(np.mean([np.linalg.norm(v)
                                          for v in t._vel_history[-10:]])) \
                          if len(t._vel_history) >= 3 else 0.0
            n_contacts  = len(self._contact_log[t.tid])
            bc_val      = float(bc.get(t.tid, 0.0))
            pol_anom    = t.pol.anomaly_score(ts, t.pos) if t.pol._fitted else 0.5

            # Dwell ratio
            if len(t._pos_history) >= 5:
                ph = np.array(list(t._pos_history)[-20:])
                median_pos = np.median(ph, axis=0)
                dwell_r = float(np.mean(np.linalg.norm(ph - median_pos, axis=1)
                                         < p.coloc_dist_m * 0.5))
            else:
                dwell_r = 0.5

            # Use relative batch classification
            role = batch_roles.get(t.tid, 'UNKNOWN')

            self._role_history[t.tid].append(role)
            self._role_history[t.tid] = self._role_history[t.tid][-10:]

            # Stable role = same classification for last 3+ scans
            hist = self._role_history[t.tid]
            if len(hist) >= 3 and all(r == role for r in hist[-3:]) \
                    and role != 'UNKNOWN':
                events.append({
                    'type':           'NETWORK_ROLE',
                    'track':          t.tid,
                    'role':           role,
                    'avg_speed_mps':  round(avg_speed, 2),
                    'n_contacts':     n_contacts,
                    'betweenness':    round(bc_val, 4),
                    'pol_anomaly':    round(pol_anom, 3),
                    'dwell_ratio':    round(dwell_r, 3),
                    'confidence':     self._role_confidence(hist),
                    'timestamp':      ts,
                    'severity':       ('HIGH' if role in ('HANDLER','ASSET')
                                       else 'MEDIUM'),
                })

        return events

    def _classify_batch(self, tracks: List) -> Dict[str, str]:
        """
        Relative classification within the current track set.
        Percentile-based so it works regardless of absolute contact counts
        (which vary with scenario density).
        """
        if len(tracks) < 2:
            return {t.tid: 'UNKNOWN' for t in tracks}

        p = self.profile
        tids     = [t.tid for t in tracks]
        speeds   = np.array([np.mean([np.linalg.norm(v) for v in t._vel_history[-10:]])
                              if len(t._vel_history) >= 3 else 0.0 for t in tracks])
        contacts = np.array([len(self._contact_log[t.tid]) for t in tracks])
        pol_anoms = np.array([t.pol.anomaly_score(0, t.pos) if t.pol._fitted else 0.5
                               for t in tracks])

        # Percentile ranks (0=lowest, 1=highest)
        def prank(arr):
            n = len(arr)
            if n <= 1: return np.zeros(n)
            order = arr.argsort()
            ranks = np.empty(n)
            ranks[order] = np.arange(n)
            return ranks / max(n - 1, 1)

        speed_r   = prank(speeds)
        contact_r = prank(contacts)

        roles = {}
        for idx, t in enumerate(tracks):
            if t.age < 10:
                roles[t.tid] = 'UNKNOWN'
                continue
            s_r = speed_r[idx];  c_r = contact_r[idx]
            pa  = pol_anoms[idx]

            # COURIER: top tercile of speed AND contacts
            if s_r > 0.60 and c_r > 0.55:
                roles[t.tid] = 'COURIER'
            # HANDLER: bottom tercile of speed, middle-to-high contacts, stable
            elif s_r < 0.35 and c_r > 0.30:
                roles[t.tid] = 'HANDLER'
            # ASSET: high PoL anomaly, low contacts relative to network
            elif pa > 0.62 and c_r < 0.35:
                roles[t.tid] = 'ASSET'
            else:
                roles[t.tid] = 'UNKNOWN'

        return roles

    @staticmethod
    def _classify(speed: float, n_contacts: int, bc: float, pol_anom: float,
                  dwell_r: float, p: DomainProfile) -> str:
        """Legacy single-track classifier (used as fallback)."""
        if speed >= p.courier_speed_thresh and n_contacts >= p.courier_contact_n and dwell_r < 0.4:
            return 'COURIER'
        if speed < p.courier_speed_thresh * 0.6 and n_contacts <= p.handler_contact_max and bc > 0.1 and dwell_r > 0.5:
            return 'HANDLER'
        if pol_anom > 0.65 and n_contacts <= 2 and speed < p.courier_speed_thresh * 0.8:
            return 'ASSET'
        return 'UNKNOWN'

    @staticmethod
    def _role_confidence(hist: List[str]) -> float:
        if not hist:
            return 0.0
        top = max(set(hist), key=hist.count)
        return round(hist.count(top) / len(hist), 3)


# ──────────────────────────────────────────────────────────────────────────────
#  LEGACY TRADECRAFT DETECTOR  (v5 brush pass + SDR + dead drop)
# ──────────────────────────────────────────────────────────────────────────────

class LegacyTradecraftDetector(BaseDetector):
    """Carries over v5 brush pass, SDR winding number, and dead drop."""
    name = 'LegacyTradecraft'

    def __init__(self, profile: DomainProfile):
        super().__init__(profile)
        self._visits: Dict[str, List[Tuple[float,np.ndarray]]] = defaultdict(list)
        self._rv_hist: Dict[tuple,int] = defaultdict(int)

    def detect(self, tracks: List, context: Dict) -> List[Dict]:
        p      = self.profile
        ts     = context['timestamp']
        scan   = context['scan_index']
        events = []

        for t in tracks:
            self._visits[t.tid].append((ts, t.pos.copy()))
            if len(self._visits[t.tid]) > 50:
                self._visits[t.tid].pop(0)

        # Brush pass
        for i in range(len(tracks)):
            for j in range(i+1, len(tracks)):
                ta, tb = tracks[i], tracks[j]
                sep = float(np.linalg.norm(ta.pos - tb.pos))
                key = (ta.tid, tb.tid)
                if sep < p.brush_pass_m:
                    self._rv_hist[key] = self._rv_hist.get(key,0) + 1
                    if self._rv_hist[key] == 1:
                        events.append({'type':'BRUSH_PASS','tracks':[ta.tid,tb.tid],
                                       'sep_m':round(sep,1),'timestamp':ts,'severity':'HIGH'})
                else:
                    if self._rv_hist.get(key,0) >= 1:
                        self._rv_hist[key] = 0

        # SDR winding number
        for t in tracks:
            vis = self._visits.get(t.tid,[])
            if len(vis) < 8: continue
            pts = np.array([v[1] for v in vis[-12:]])
            c = pts.mean(0); d = pts - c
            angles = np.arctan2(d[:,1],d[:,0])
            wn = abs(np.diff(np.unwrap(angles)).sum())/(2*np.pi)
            if wn >= 0.65:
                events.append({'type':'SDR_PATTERN','track':t.tid,
                               'winding_number':round(float(wn),2),
                               'timestamp':ts,'severity':'HIGH'})

        # Dead drop (throttled)
        if len(tracks) >= 2 and scan % 3 == 0:
            loc_vis: Dict[str,List] = defaultdict(list)
            for t in tracks:
                for vts, vpos in self._visits.get(t.tid,[])[-5:]:
                    cell = f"{int(vpos[0]/200)}_{int(vpos[1]/200)}"
                    loc_vis[cell].append((t.tid, vts))
            for cell, visitors in loc_vis.items():
                tids = list(set(v[0] for v in visitors))
                if len(tids) < 2: continue
                times = [v[1] for v in visitors]
                spread = max(times)-min(times)
                lo, hi = p.dead_drop_spread
                if lo < spread < hi:
                    simult = sum(1 for a,at in visitors for b,bt in visitors
                                 if a!=b and abs(at-bt)<30)
                    if simult == 0:
                        events.append({'type':'DEAD_DROP','tracks':tids[:3],'cell':cell,
                                       'time_spread_s':round(spread,0),
                                       'timestamp':ts,'severity':'IMMEDIATE'})
        return events


# ──────────────────────────────────────────────────────────────────────────────
#  v6 ARIA-INTEL ENGINE  (domain-polymorphic, composable detectors)
# ──────────────────────────────────────────────────────────────────────────────

# Import v5 internals we're reusing directly
_V5_NAMES = [
    'Observation','PatternOfLife','MOUParticleFilter','BernoulliTrack',
    'AdaptiveClutterEstimator','SourceCredibilityTracker','GibbsAssigner',
    'PMBMManager','score_track','_priority',
    'CredibilityFuser','SensorScheduler','OperationalIntelligence',
    'ForwardBackwardSmoother','DynamicNetworkAnalyser','AnomalyEscalator',
    'RoutePredictor','generate_scenario',
    '_MOU_ALPHA','_MOU_SIG_V','_MOU_SS_VVAR','MODEL_KEYS','N_MODELS',
    'N_PARTICLES','MODALITY_WEIGHT','TIER',
    'logsumexp','_fwdsub3','_gmm_logpdf_batch','_betweenness_centrality',
    '_chol_logpdf',
]


class ARIAIntelEngineV6:
    """
    v6 Engine: domain-polymorphic, composable detector pipeline.

    Usage:
        eng = ARIAIntelEngineV6(profile=Maritime(), area=(...), hvls=[...])
        # or
        eng = ARIAIntelEngineV6()  # defaults to UrbanHUMINT

    Custom detectors:
        eng.register_detector(MyCustomDetector(eng.profile))
        eng.unregister_detector('MyCustomDetector')
    """

    def __init__(self,
                 profile: Optional[DomainProfile] = None,
                 area: Tuple[float,float,float,float] = (-5000,5000,-5000,5000),
                 high_value_locations: Optional[List[np.ndarray]] = None,
                 hvl_radius: float = None):

        self.profile = profile or UrbanHUMINT()
        p = self.profile

        self.pmbm        = PMBMManager(area)
        self.network     = DynamicNetworkAnalyser(coloc_dist=p.coloc_dist_m)
        self.escalator   = AnomalyEscalator()
        self.credibility = CredibilityFuser()
        self.scheduler   = SensorScheduler()
        self.opint       = OperationalIntelligence()
        self.smoother    = ForwardBackwardSmoother(lag=6)
        self.router      = RoutePredictor()

        self.hvls   = high_value_locations or []
        self.hvl_r  = hvl_radius or p.hvl_radius_m

        # ── Composable detector registry ──────────────────────────────────
        self._detectors: Dict[str, BaseDetector] = {}
        self._register_default_detectors()

        self.scan_count  = 0
        self.all_reports: List[Dict] = []
        self._obs_cache: Dict[str, List] = defaultdict(list)
        self._sep_history: Dict[Tuple,List] = defaultdict(list)

    def _register_default_detectors(self):
        p = self.profile
        self.register_detector(LegacyTradecraftDetector(p))
        self.register_detector(ExtendedRendezvousWarner(p))
        self.register_detector(ParallelRouteSurveillanceDetector(p))
        self.register_detector(ModeTransitionDetector(p))
        self.register_detector(LoiterAnomalyDetector(p))
        self.register_detector(CoverStopDetector(p))
        self.register_detector(ChokepointSurveillanceDetector(p))
        self.register_detector(NetworkRoleInference(p))

    def register_detector(self, detector: BaseDetector):
        self._detectors[detector.name] = detector

    def unregister_detector(self, name: str):
        self._detectors.pop(name, None)

    def list_detectors(self) -> List[str]:
        return list(self._detectors.keys())

    def ingest(self, observations: List, timestamp: float,
               forecast_horizon: int = 6) -> Dict:
        self.scan_count += 1
        self.pmbm.predict()
        self.pmbm.update(observations, timestamp)
        confirmed = self.pmbm.confirmed()

        valid = [o for o in observations if o.position is not None]
        for t in confirmed:
            for obs in valid:
                if np.linalg.norm(obs.position - t.pos) < 120:
                    self._obs_cache[t.tid].append(obs)
            self._obs_cache[t.tid] = self._obs_cache[t.tid][-20:]
            self.smoother.update(t.tid, t.pf.X, t.pf.w)

        # Score all confirmed tracks
        targets = []
        for t in confirmed:
            sc   = score_track(t, timestamp, self.hvls, self.hvl_r)
            cred = self.credibility.combine(self._obs_cache[t.tid])
            esc  = self.escalator.update(t.tid, sc['breakdown']['pol_anomaly'])
            alerts_t = [{'tier':sc['priority'],'track':t.tid,
                          'reason':r,'score':s} for r,s in esc]
            forecast = []
            if sc['priority'] in ('IMMEDIATE','HIGH'):
                forecast = self.router.forecast(t, horizon=forecast_horizon,
                                                 dt_per_scan=self.profile.scan_dt_s)
            smoothed = self.smoother.smooth_pos(t.tid)
            targets.append({
                'track_id':       t.tid,
                'parent_id':      t.parent_id,
                'born_at':        round(t.born_at,0),
                'last_seen':      round(t.last_seen,0),
                'position':       t.pos.tolist(),
                'smoothed_pos':   smoothed.tolist() if smoothed is not None else t.pos.tolist(),
                'velocity':       t.vel.tolist(),
                'speed_mps':      round(float(np.linalg.norm(t.vel)),2),
                'pos_uncertainty_m': round(t.pos_uncertainty(),1),
                'existence_p':    round(t.r,4),
                'poss_existence': round(t.pi_r,4),
                'age_scans':      t.age,
                'n_hits':         t.n_hit,
                'n_misses':       t.n_miss,
                'obs_quality':    round(t.mean_obs_quality,3),
                'meas_rate':      round(t.mrate,3),
                'credibility':    cred,
                'forecast':       forecast,
                '_alerts':        alerts_t,
                **sc,
            })
        targets.sort(key=lambda x: -x['threat_score_mean'])
        all_alerts = [a for t in targets for a in t.pop('_alerts')]

        # Network analysis (provides betweenness for role inference)
        clusters  = self.network.analyse(confirmed, timestamp)
        bc_map    = {}
        for cl in clusters:
            for tid, bv in cl['betweenness'].items():
                bc_map[tid] = bv

        # Run all registered detectors
        context = {
            'timestamp':   timestamp,
            'scan_index':  self.scan_count,
            'hvls':        self.hvls,
            'profile':     self.profile,
            'betweenness': bc_map,
            'clusters':    clusters,
        }
        all_detections: Dict[str, List[Dict]] = {}
        for dname, det in self._detectors.items():
            try:
                all_detections[dname] = det.detect(confirmed, context)
            except Exception as e:
                all_detections[dname] = [{'type':'DETECTOR_ERROR','detector':dname,
                                           'error':str(e)}]

        # Flatten by category for backward compat + new fields
        tradecraft_events = (all_detections.get('LegacyTradecraft', []) +
                             all_detections.get('ParallelRouteSurveillance', []) +
                             all_detections.get('ModeTransition', []) +
                             all_detections.get('LoiterAnomaly', []) +
                             all_detections.get('CoverStop', []) +
                             all_detections.get('ChokepointSurveillance', []))
        rv_events     = all_detections.get('ExtendedRendezvousWarner', [])
        role_events   = all_detections.get('NetworkRoleInference', [])
        sched         = self.scheduler.schedule(confirmed, list(MODALITY_WEIGHT.keys()))
        op_intel      = self.opint.analyse(confirmed, timestamp)

        report = {
            'scan':           self.scan_count,
            'timestamp':      timestamp,
            'domain':         self.profile.name,
            'n_obs':          len(observations),
            'n_tracks':       len(confirmed),
            'n_components':   len(self.pmbm.tracks),
            'n_dormant':      len(self.pmbm.dormant),
            'clutter_rate':   round(self.pmbm.clutter.rate, 2),
            'targets':        targets,
            'rendezvous':     rv_events,
            'clusters':       clusters,
            'tradecraft':     tradecraft_events,
            'network_roles':  role_events,
            'alerts':         all_alerts,
            'sensor_schedule': sched[:3],
            'operational':    op_intel,
            'all_detections': all_detections,
        }
        self.all_reports.append(report)
        return report

    def summary(self, r: Dict) -> str:
        lines = [
            f"╔══ ARIA-INTEL v6 [{r['domain']}] | SCAN {r['scan']:04d} | "
            f"t={r['timestamp']:.0f}s | clutter={r['clutter_rate']:.1f} | "
            f"dormant={r['n_dormant']} ══╗",
            f"  Obs:{r['n_obs']}  Confirmed:{r['n_tracks']}  "
            f"Components:{r['n_components']}",
        ]
        if r['alerts']:
            lines.append("  ⚠  " + " | ".join(
                f"{a['reason']}({a['track']})" for a in r['alerts'][:5]))

        # Tradecraft events grouped by type
        tc_by_type: Dict[str,List] = defaultdict(list)
        for e in r['tradecraft']:
            tc_by_type[e['type']].append(e)
        for etype, evs in tc_by_type.items():
            trk = evs[0].get('track', evs[0].get('tracks', evs[0].get('subject','?')))
            lines.append(f"  🔴 {etype}({trk}) ×{len(evs)}  sev={evs[0].get('severity','?')}")

        # Rendezvous warnings
        if r['rendezvous']:
            lines.append("  ── RENDEZVOUS WARNINGS ──")
            for rv in r['rendezvous'][:6]:
                lines.append(
                    f"  {rv['track_a']}↔{rv['track_b']}  "
                    f"ETA={rv['eta_min']:.1f}min  "
                    f"sep={rv['current_sep_m']:.0f}m  "
                    f"method={rv['method']}  "
                    f"conf={rv['confidence']:.2f}  {rv['priority']}")

        # Network roles
        if r['network_roles']:
            lines.append("  ── NETWORK ROLES ──")
            for nr in r['network_roles'][:5]:
                lines.append(
                    f"  {nr['track']}  role={nr['role']}  "
                    f"contacts={nr['n_contacts']}  "
                    f"spd={nr['avg_speed_mps']:.1f}  "
                    f"conf={nr['confidence']:.2f}  {nr['severity']}")

        # Track table
        lines.append(f"  {'ID':<7} {'Pri':<10} {'Score':>9} "
                     f"{'Exist':>6} {'Pos':>22} {'Model':>10} {'MR':>5}")
        lines.append("  " + "─" * 88)
        for t in r['targets']:
            px, py = t['position']
            bd = t['breakdown']
            lines.append(
                f"  {t['track_id']:<7} {t['priority']:<10} "
                f"{t['threat_score_mean']:.3f}±{t['threat_score_std']:.3f}  "
                f"{t['existence_p']:>5.3f}  [{px:+7.0f},{py:+7.0f}]  "
                f"{t['dominant_model']:>10}  {t['meas_rate']:>4.2f}")

        if r['clusters']:
            lines.append("  ── NETWORK (betweenness) ──")
            for cl in r['clusters'][:3]:
                lines.append(
                    f"  C{cl['cluster_id']}: {cl['member_ids']}  "
                    f"hub={cl['hub_track']}  recurring={cl['recurring']}")

        lines.append("╚" + "═" * 70)
        return "\n".join(lines)

    def performance_report(self) -> str:
        if not self.all_reports: return "No data."
        peak  = max(r['n_tracks'] for r in self.all_reports)
        ids   = {t['track_id'] for r in self.all_reports for t in r['targets']}
        tc    = sum(len(r['tradecraft']) for r in self.all_reports)
        rv    = sum(len(r['rendezvous']) for r in self.all_reports)
        roles = sum(len(r['network_roles']) for r in self.all_reports)
        mm    = sum(len(r['operational'].get('model_transitions',[]))
                    for r in self.all_reports)
        return (f"Domain:{self.profile.name}  Scans:{self.scan_count}  "
                f"PeakTracks:{peak}  UniqueTracks:{len(ids)}  "
                f"Tradecraft:{tc}  RV_Warnings:{rv}  "
                f"NetworkRoles:{roles}  PossMismatch:{mm}")
def generate_scenario(n_scans: int = 35, n_targets: int = 7,
                       area: float = 4000.0, seed: int = 77):
    rng = np.random.RandomState(seed)
    MODS = ['GEOINT','SIGINT','COMMS','HUMINT','OSINT']
    MOD_CONF = {'GEOINT':(0.88,0.10),'SIGINT':(0.76,0.14),'COMMS':(0.70,0.15),
                'HUMINT':(0.60,0.17),'OSINT':(0.50,0.18)}
    states = []
    for _ in range(n_targets):
        x = rng.uniform(-area*0.6, area*0.6, 4); x[2:] = rng.uniform(-10,10,2)
        states.append(x)
    F_gen = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], dtype=np.float64)
    Q_gen = np.diag([1.0,1.0,2.5,2.5])
    true_traj = np.zeros((n_scans, n_targets, 4))
    all_obs = []
    for scan in range(n_scans):
        t = float(scan*60); scan_obs = []
        for j in range(n_targets):
            if rng.random() < 0.08: states[j][2:] += rng.normal(0,6,2); states[j][2:] = np.clip(states[j][2:],-18,18)
            states[j] = F_gen @ states[j] + rng.multivariate_normal(np.zeros(4), Q_gen*0.25)
            for dim in range(2):
                for bound, sign in [(-area,-1),(area,1)]:
                    if sign*states[j][dim] > sign*bound:
                        states[j][dim] = bound*0.9; states[j][dim+2] *= -0.7
            true_traj[scan,j] = states[j]
            if rng.random() < P_DETECTION:
                mod = rng.choice(MODS); mu,sg = MOD_CONF[mod]
                conf = float(np.clip(rng.normal(mu,sg),0.1,1.0))
                pos  = states[j][:2] + rng.multivariate_normal(np.zeros(2), R)
                src  = f"SRC_{mod}_{rng.randint(0,5)}"
                scan_obs.append(Observation(uuid.uuid4().hex[:8], t, pos, mod, conf, src))
        for _ in range(rng.poisson(3.0)):
            scan_obs.append(Observation(uuid.uuid4().hex[:8], t,
                rng.uniform(-area,area,2), 'OSINT',
                float(rng.uniform(0.05,0.30)), f"SRC_OSINT_{rng.randint(0,3)}"))
        rng.shuffle(scan_obs); all_obs.append(scan_obs)
    return all_obs, true_traj



