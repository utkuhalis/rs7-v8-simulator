import math
import numpy as np
import pygame
import sounddevice as sd
from scipy import signal

pygame.init()

WIDTH, HEIGHT = 920, 660
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Audi RS7 4.0 TFSI V8 Biturbo")
font = pygame.font.SysFont(None, 48)
clock = pygame.time.Clock()

# -------------------- Arac durumu --------------------
rpm = 900.0
speed = 0.0
gear = 1
throttle = 0.0
pop_burst = 0.0      # ani egzoz patlamasi tetigi (lift/vites/limiter)
bov_burst = 0.0      # blow-off valf tetigi (boost altinda gaz kesince)
auto = True          # otomatik vites
exhaust_mode = True  # True=EGZOZ (disardan), False=MOTOR (kabin)
brake = 0.0
engine_on = False    # motor calisiyor mu
cranking = False     # mars donuyor mu
dying = False        # motor sonuyor mu (devir dususu)
die_t = 0.0          # sonme sayaci
dying_rpm0 = 0.0     # sonme baslangic devri
engine_temp = 0.0    # 0=soguk 1=sicak (fast-idle + kaba rolanti)
start_flare = 0.0    # calisma aninda kisa vroom
engine_level = 0.0   # ses seviyesi zarfi (mars/sonme gecisleri)
limiter = False      # devir limitine vuruyor mu (sert kesme)
wheelspin = 0.0      # patinaj miktari (0..1) -> lastik cizirtisi
g_smooth = 0.0       # yumusatilmis boyuna G kuvveti (g-metre)
g_lat_smooth = 0.0   # yumusatilmis yanal G (viraj)
steer = 0.0          # direksiyon (-1..1)
drifting = False     # kayma / drift durumu
wheel_ang = 0.0      # tekerlek donme acisi (animasyon)
theme_idx = 0        # renk temasi (skin)
telem_rpm, telem_spd, telem_g = [], [], []   # telemetri gecmisi
driveby = False      # yanindan gecis (drive-by) animasyonu
db_x = 0.0           # sanal arac konumu (m)
doppler = 1.0        # doppler frekans carpani
db_lg = 1.0          # sol kanal kazanci (pan)
db_rg = 1.0          # sag kanal kazanci (pan)
db_vol = 1.0         # mesafe ses zarfi
flame = 0.0          # egzoz alevi parlamasi (0..1), BANG'de tetiklenir
stage = 0            # 0=STOCK, 1/2/3 = ECU tuning stage
STAGE_MUL = [1.0, 1.22, 1.42, 1.65]            # tork carpani
STAGE_NAMES = ["STOCK", "STAGE 1", "STAGE 2", "STAGE 3"]
STAGE_HP = [600, 700, 800, 1000]               # gosterge icin (~hp)

gear_ratios = {1: 4.71, 2: 3.14, 3: 2.10, 4: 1.67,
               5: 1.29, 6: 1.00, 7: 0.84, 8: 0.67}

IDLE_RPM = 820.0
MAX_RPM = 7000.0
REDLINE = 6800.0

# -------------------- Ses --------------------
SR = 44100
BLOCK = 512

# Cross-plane V8: 2 turda (1 cevrim) 8 atesleme.
# Banka genlik farki + hafif duzensizlik = RS7'nin "lope/burble" karakteri.
_base = np.arange(8) / 8.0
_jit = np.array([0, .010, 0, .010, 0, .010, 0, .010])
FIRE_ANGLES = (_base + _jit) % 1.0
FIRE_AMPS = np.array([1.0, .72, 1.0, .72, 1.0, .72, 1.0, .72])


def rbj_bandpass(f, Q):
    """RBJ bandpass biquad (peak gain = Q) -> rezonator."""
    w0 = 2 * math.pi * f / SR
    a = math.sin(w0) / (2 * Q)
    cw = math.cos(w0)
    b = np.array([a, 0.0, -a])
    aa = np.array([1 + a, -2 * cw, 1 - a])
    return b / aa[0], aa / aa[0]


# Egzoz "govdesi": sabit rezonanslar. Atesleme hizi devirle artar ama
# rezonans frekanslari (boru geometrisi) sabit -> gercek egzoz boyle davranir.
RESONATORS = [
    dict(f=70,   Q=5,  base=1.00, load=0.00),   # alt homurtu / thump
    dict(f=155,  Q=7,  base=0.90, load=0.10),
    dict(f=330,  Q=8,  base=0.65, load=0.35),   # govde
    dict(f=620,  Q=10, base=0.35, load=0.55),
    dict(f=1250, Q=14, base=0.12, load=0.85),   # rasp (gazla acilir)
    dict(f=2200, Q=20, base=0.05, load=1.00),   # metalik cin (yuksek-Q)
    dict(f=3400, Q=24, base=0.02, load=0.80),   # metalik tiz rasp
]
for r in RESONATORS:
    r["b"], r["a"] = rbj_bandpass(r["f"], r["Q"])
    r["zi"] = np.zeros(2)


def make_comb(freq, g):
    # geri beslemeli comb (waveguide) -> egzoz borusu metalik rezonansi
    D = int(round(SR / freq))
    a = np.zeros(D + 1); a[0] = 1.0; a[D] = -g
    b = np.zeros(D + 1); b[0] = 1.0
    return b, a, D


COMB_B, COMB_A, COMB_D = make_comb(190.0, 0.80)   # ~190 Hz boru rezonansi
# combine giren metalik tiz kismi icin high-pass
HP_B, HP_A = signal.butter(2, 700 / (SR / 2), btype="high")

# Sub-bass (gogus thump) - derin alt frekanslar
SUB_B, SUB_A = signal.butter(2, 95 / (SR / 2), btype="low")
# Ruzgar / yol gurultusu (hizla artar, stereo dekorele)
WIND_B, WIND_A = signal.butter(2, [220 / (SR / 2), 3200 / (SR / 2)], btype="band")
# Intake / induction gurultusu
NZ_B, NZ_A = signal.butter(2, [500 / (SR / 2), 2200 / (SR / 2)], btype="band")
# Genel cikis lowpass (sertligi alir)
OUT_B, OUT_A = signal.butter(2, 7500 / (SR / 2), btype="low")
# "Motor" modu icin ek bogma lowpass (egzoz modunda atlanir)
MUF_B, MUF_A = signal.butter(2, 1900 / (SR / 2), btype="low")
# Blow-off / dump valf "psshh" (genis bantli hava gurultusu)
BOV_B, BOV_A = signal.butter(2, [1200 / (SR / 2), 6500 / (SR / 2)], btype="band")
# Lastik cizirtisi (patinaj) - rezonant squeal
TIRE_B, TIRE_A = rbj_bandpass(680, 8)

# -------------------- Egzoz patlama cekirdekleri --------------------
# Her "pat" = keskin atak + alt thump + parlak catirti, ustel sonum.
# Tek-sample impuls yerine bunlar gercek patlama gibi duyulur.
POP_LEN = int(0.060 * SR)


def make_pop_kernel(c):
    t = np.arange(POP_LEN) / SR
    env = np.exp(-t * (40 + 25 * c))
    thump = np.sin(2 * np.pi * (85 + 50 * c) * t) * 0.8        # alt patlama
    mid = np.sin(2 * np.pi * (480 + 380 * c) * t) * np.exp(-t * 80) * 0.5
    crack = np.random.randn(POP_LEN) * np.exp(-t * 130) * 1.3  # parlak catirti
    k = (thump + mid + crack) * env
    return (k / (np.max(np.abs(k)) + 1e-9)).astype(np.float64)


POP_KERNELS = [make_pop_kernel(i / 5.0) for i in range(6)]

# Buyuk anti-lag patlamasi (BANG) - alt boom + catirti
BOOM_LEN = int(0.18 * SR)


def make_boom():
    t = np.arange(BOOM_LEN) / SR
    env = np.exp(-t * 15)
    boom = (np.sin(2 * np.pi * 55 * t) + 0.5 * np.sin(2 * np.pi * 90 * t)) * env
    crack = np.random.randn(BOOM_LEN) * np.exp(-t * 55) * 0.6
    k = boom + crack
    return (k / (np.max(np.abs(k)) + 1e-9)).astype(np.float64)


BOOM_KERNEL = make_boom()


class Engine:
    def __init__(self):
        self.cyc = 0.0          # monoton cevrim sayaci
        self.s_rpm = IDLE_RPM
        self.s_thr = 0.0
        self.crackle = 0.0
        self.turbo_ph = 0.0
        # zi uzunluklari filtre derecesinden (len(a)-1) gelir
        self.zi_sub = np.zeros(len(SUB_A) - 1)
        self.zi_nz = np.zeros(len(NZ_A) - 1)
        self.zi_out = np.zeros(len(OUT_A) - 1)
        self.zi_muf = np.zeros(len(MUF_A) - 1)
        self.zi_bov = np.zeros(len(BOV_A) - 1)
        self.pop_tail = np.zeros(POP_LEN)   # bloklar arasi patlama kuyrugu
        self.boost = 0.0                    # turbo basinci (0..1), gecikmeli spool
        self.bov = 0.0                      # blow-off zarf
        self.lim_ph = 0.0                   # rev-limiter kesme fazi
        self.zi_windL = np.zeros(len(WIND_A) - 1)
        self.zi_windR = np.zeros(len(WIND_A) - 1)
        self.dly_tail = np.zeros(14)        # stereo genislik (Haas) gecikmesi
        self.zi_tire = np.zeros(len(TIRE_A) - 1)
        self.tire_ph = 0.0                  # lastik squeal tonu fazi
        self.zi_hp = np.zeros(len(HP_A) - 1)
        self.zi_comb = np.zeros(COMB_D)     # comb (boru) gecikme hatti
        self.boom_tail = np.zeros(BOOM_LEN)  # buyuk BANG kuyrugu
        self.idx = np.arange(BLOCK)

    def callback(self, out, frames, t, status):
        # --- motor kapali: sessizlik (engine_level zarfi mars/sonmeyi yonetir) ---
        if engine_level <= 0.001:
            out[:] = 0.0
            return

        # --- parametre yumusatma ---
        self.s_rpm += (rpm - self.s_rpm) * 0.30
        self.s_thr += (throttle - self.s_thr) * 0.25
        r = self.s_rpm
        thr = self.s_thr
        rn = np.clip((r - IDLE_RPM) / (MAX_RPM - IDLE_RPM), 0, 1)

        cyc_freq = r / 120.0 * doppler   # cevrim/saniye (2 tur) * doppler
        inc = cyc_freq / SR
        span = frames * inc

        # --- atesleme darbe treni ---
        imp = np.zeros(frames)
        for a, amp in zip(FIRE_ANGLES, FIRE_AMPS):
            n0 = math.ceil(self.cyc - a)
            n1 = math.floor(self.cyc + span - a - 1e-9)
            for n in range(n0, n1 + 1):
                si = int(round((n + a - self.cyc) / inc))
                if 0 <= si < frames:
                    # her yanma biraz farkli -> canlilik
                    imp[si] += amp * (0.8 + 0.4 * np.random.rand())
        self.cyc += span
        if self.cyc > 1e6:
            self.cyc -= 1e6

        # --- govde: darbeler -> rezonans bankasi ---
        body = np.zeros(frames)
        for res in RESONATORS:
            y, res["zi"] = signal.lfilter(res["b"], res["a"], imp, zi=res["zi"])
            w = res["base"] + res["load"] * (0.3 + 0.7 * thr)
            body += w * y

        # --- metalik egzoz borusu rezonansi (comb/waveguide) ---
        # govdenin tiz kismini geri-beslemeli comb'dan gecir -> metalik "cin"
        metal_in, self.zi_hp = signal.lfilter(HP_B, HP_A, body, zi=self.zi_hp)
        metal, self.zi_comb = signal.lfilter(COMB_B, COMB_A, metal_in, zi=self.zi_comb)
        metal_gain = (0.30 + 0.55 * thr) * (1.0 if exhaust_mode else 0.4)

        # --- sub-bass thump (derin gogus frekanslari) ---
        sub, self.zi_sub = signal.lfilter(SUB_B, SUB_A, imp, zi=self.zi_sub)
        sub *= 2.7

        sig = 4.4 * body + 2.3 * sub + metal * metal_gain

        # --- egzoz patlamalari: gercek "pat" cekirdekleri, overlap-add ---
        global pop_burst
        if thr < 0.30 and r > 2200:
            self.crackle = min(1.0, self.crackle + 0.12)
        else:
            self.crackle *= 0.84
        ov = self.crackle * np.clip((r - 2000) / 3000, 0, 1)
        burst = pop_burst           # ani patlama (lift / vites / limiter)
        pop_burst *= 0.80
        intensity = max(ov, burst)

        work = np.zeros(frames + POP_LEN)
        work[:POP_LEN] += self.pop_tail
        if intensity > 0.01:
            # patlama yogunlugu: overrun seyrek "pat..pat", burst sik "patpatpat"
            # yuksek stage -> daha cok anti-lag patlama
            rate = (18.0 * ov + 75.0 * burst) * (1.0 + 0.35 * stage)
            n_pop = np.random.poisson(rate * frames / SR)
            for _ in range(int(n_pop)):
                p = np.random.randint(0, frames)
                amp = (0.55 + 0.6 * np.random.rand()) * (0.5 + 0.8 * intensity)
                k = POP_KERNELS[np.random.randint(len(POP_KERNELS))]
                work[p:p + POP_LEN] += k * amp
        pop_out = work[:frames]
        self.pop_tail = work[frames:frames + POP_LEN]

        # ses modu: EGZOZ = patlamalar one cikar, MOTOR = boguk/mekanik
        pop_gain = (0.9 if exhaust_mode else 0.30) * (1.0 + 0.18 * stage)
        sig += pop_out * pop_gain

        # --- buyuk anti-lag BANG + alev (lift sirasinda, ara sira) ---
        global flame
        work_b = np.zeros(frames + BOOM_LEN)
        work_b[:BOOM_LEN] += self.boom_tail
        if burst > 0.4 and r > 3000 and np.random.rand() < (0.30 + 0.16 * stage):
            p = np.random.randint(0, frames)
            work_b[p:p + BOOM_LEN] += BOOM_KERNEL * (0.8 + 0.5 * np.random.rand())
            flame = 1.0                        # gorsel alev tetigi
        boom_out = work_b[:frames]
        self.boom_tail = work_b[frames:frames + BOOM_LEN]
        sig += boom_out * (1.3 if exhaust_mode else 0.5)

        # --- intake / induction noise (motor modunda daha belirgin) ---
        noise = np.random.randn(frames)
        nz, self.zi_nz = signal.lfilter(NZ_B, NZ_A, noise, zi=self.zi_nz)
        nz_gain = (0.04 + 0.20 * thr) * (0.7 if exhaust_mode else 1.6)
        sig += nz * nz_gain

        # --- turbo spool (gecikmeli basinc) + whine ---
        global bov_burst
        btar = thr * float(np.clip((r - 1500) / 2500, 0, 1))
        # yukselis yavas (spool), dususte daha hizli
        self.boost += (btar - self.boost) * (0.030 if btar > self.boost else 0.10)
        tf = 1200 + self.boost * 2600 + rn * 3200      # boost & devirle tizleşir
        ti = tf / SR
        tph = self.turbo_ph + self.idx * ti
        self.turbo_ph = (self.turbo_ph + frames * ti) % 1.0
        tgain = (0.05 if exhaust_mode else 0.11) * (0.25 + 0.9 * self.boost) * (1.0 + 0.25 * stage)
        sig += np.sin(2 * np.pi * tph) * tgain

        # --- blow-off / dump valf "psshh" (boost varken gaz kesilince) ---
        self.bov = max(self.bov * 0.86, bov_burst)
        bov_burst *= 0.4
        if self.bov > 0.02:
            bn = np.random.randn(frames)
            bn, self.zi_bov = signal.lfilter(BOV_B, BOV_A, bn, zi=self.zi_bov)
            sig += bn * self.bov * (0.7 if exhaust_mode else 0.9)

        # --- lastik cizirtisi (patinaj): rezonant squeal + gurultu ---
        if wheelspin > 0.02:
            sf = (560 + 120 * wheelspin) / SR      # patinajla tizlesen squeal
            sph = self.tire_ph + self.idx * sf
            self.tire_ph = (self.tire_ph + frames * sf) % 1.0
            tn, self.zi_tire = signal.lfilter(TIRE_B, TIRE_A, np.random.randn(frames), zi=self.zi_tire)
            squeal = 0.45 * np.sin(2 * np.pi * sph) + 0.6 * tn
            sig += squeal * wheelspin * 0.55

        # --- mod bogma: MOTOR modunda ekstra lowpass (kabin/mekanik his) ---
        if not exhaust_mode:
            sig, self.zi_muf = signal.lfilter(MUF_B, MUF_A, sig, zi=self.zi_muf)

        # --- rev-limiter: motoru ritmik kes ("dut-dut-dut") ---
        if limiter:
            lf = 17.0 / SR                       # ~17 Hz kesme
            lph = self.lim_ph + self.idx * lf
            self.lim_ph = (self.lim_ph + frames * lf) % 1.0
            gate = 0.18 + 0.82 * (np.sin(2 * np.pi * lph) > -0.1)
            sig *= gate

        # --- mars motoru: elektrikli vinlamasi + duzensiz chug ---
        if cranking:
            stf = 110.0 / SR                     # marş elektrik motoru
            sph = self.turbo_ph * 4 + self.idx * stf
            whir = (signal.sawtooth(2 * np.pi * sph) * 0.5
                    + 0.5 * np.sin(2 * np.pi * sph * 2)) * 0.13
            whir *= (0.6 + 0.4 * np.random.rand(frames))
            sig = sig * 0.5 + whir               # chug + marş

        # --- cikis lowpass + yumusak limit (tanh = egzoz griftligi) ---
        sig, self.zi_out = signal.lfilter(OUT_B, OUT_A, sig, zi=self.zi_out)
        master = 0.62 if exhaust_mode else 0.55
        drive = (1.8 + 1.0 * thr) if exhaust_mode else (1.3 + 0.6 * thr)
        mono = np.tanh(sig * drive) * master
        mono = np.tanh(reverb.process(mono))   # ortam akustigi + guvenli limit

        # --- stereo genislik: R kanali kucuk gecikme (Haas) ---
        delayed = np.concatenate([self.dly_tail, mono])
        right_eng = delayed[:frames]
        self.dly_tail = mono[-14:]

        # --- ruzgar / yol gurultusu (hizla artar, stereo dekorele) ---
        wind_amt = float(np.clip(speed / 260.0, 0, 1)) ** 1.3 * 0.30
        wL, self.zi_windL = signal.lfilter(WIND_B, WIND_A, np.random.randn(frames), zi=self.zi_windL)
        wR, self.zi_windR = signal.lfilter(WIND_B, WIND_A, np.random.randn(frames), zi=self.zi_windR)

        # engine_level: mars/sonme; db_*: drive-by pan + mesafe sesi
        lvl = engine_level * db_vol
        out[:, 0] = ((mono + wL * wind_amt) * lvl * db_lg).astype(np.float32)
        out[:, 1] = ((right_eng + wR * wind_amt) * lvl * db_rg).astype(np.float32)


class BlockReverb:
    """Blok-granuler geri beslemeli comb reverb (hizli). Ortam akustigi."""
    ENV_NAMES = ["ACIK", "GARAJ", "TUNEL"]

    def __init__(self):
        self.taps = [3, 4, 5, 7]      # gecikme (blok cinsinden)
        self.hist = {t: [np.zeros(BLOCK) for _ in range(t)] for t in set(self.taps)}
        self.set_env(0)

    def set_env(self, e):
        self.env = e % 3
        self.fb = [0.0, 0.50, 0.66][self.env]
        self.wet = [0.0, 0.24, 0.40][self.env]

    def process(self, x):
        if self.wet <= 0 or len(x) != BLOCK:
            return x
        wet = np.zeros(BLOCK)
        for t in self.taps:
            y = x + self.fb * self.hist[t][0]
            self.hist[t].append(y)
            self.hist[t].pop(0)
            wet += y
        wet /= len(self.taps)
        return x * (1 - 0.4 * self.wet) + wet * self.wet


reverb = BlockReverb()
eng = Engine()
stream = sd.OutputStream(samplerate=SR, blocksize=BLOCK, channels=2,
                         dtype="float32", callback=eng.callback)
stream.start()

# -------------------- Fizik (gercek RS7 C8 verileri) --------------------
# 4.0 TFSI V8: 800 Nm (2050-4500 rpm duz), ~6800 devir limiti,
# 0-100 ~3.6s, Vmax ~305 km/h (dynamic plus). 8 ileri otomatik.
TIRE_CIRC = 2.16                 # m (275/35 R21 cevre)
WHEEL_R = TIRE_CIRC / (2 * math.pi)
FINAL = 3.7                      # diferansiyel orani
MASS = 2075.0                    # kg
EFF = 0.90                       # aktarma verimi
CD, ROLL_V, ROLL0 = 0.26, 12.0, 120.0   # drag: CD*v^2 + ROLL_V*v + ROLL0
EB_TQ = 150.0                    # motor freni torku (Nm), gaz kapali iken
MU = 1.30                        # lastik tutus katsayisi (quattro AWD, sport)
GRIP = MU * MASS * 9.81          # max aktarilabilir tahrik kuvveti (N)
REV_K = 20.0                     # serbest devir ivme katsayisi (bos viteste)
REV_FR = 0.040                   # motor ic surtunmesi (devir dususu)
v = 0.0                          # m/s
rev_rpm = IDLE_RPM               # bos viteste serbest motor devri
crank_t = 0.0                    # mars suresi sayaci
idle_phase = 0.0                 # rolanti dalgalanma fazi

# performans olcumu & launch control
LAUNCH_RPM = 3900.0              # launch control devri
dist = 0.0                       # toplam mesafe (m)
accel_timer = 0.0                # mevcut hizlanma sayaci
accel_dist0 = 0.0                # hizlanma baslangic mesafesi
t100_mark = None                 # bu kosudaki 0-100 suresi
qmark = None                     # bu kosudaki ceyrek mil suresi
qtrap = 0.0                      # ceyrek mil cikis hizi
best_0_100 = None                # en iyi 0-100
best_qmile = None                # en iyi ceyrek mil (sure, trap)
timing = False
launch_rpm = IDLE_RPM            # launch sirasinda tutulan devir
launch_boost = 0.0              # kalkis sonrasi ekstra tutus zarfi
was_launching = False
QMILE = 402.336                  # ceyrek mil (m)


def pick_gear(v_ms):
    # hiza uygun en yuksek (rpm < ~5500) vitesi sec
    for g in range(1, 9):
        if rpm_from_speed(v_ms, g) < 5500:
            return g
    return 8


def torque(rp):
    # Nm: dusukte yukselir, 2050-4500 duz 800, sonra guc sinirli duser.
    # Stage (ECU tuning) torku carpan olarak artirir.
    if rp < 2050:
        base = 800.0 * (0.55 + 0.45 * rp / 2050.0)
    elif rp <= 4500:
        base = 800.0
    else:
        base = max(220.0, 800.0 * (4500.0 / rp) * 0.96)
    return base * STAGE_MUL[stage]


def rpm_from_speed(v_ms, g):
    return IDLE_RPM + (v_ms / TIRE_CIRC) * gear_ratios[g] * FINAL * 60.0


# -------------------- HUD: analog gauge cizimi --------------------
_f_small = pygame.font.SysFont("Arial", 18)
_f_med = pygame.font.SysFont("Arial", 22, bold=True)
_f_big = pygame.font.SysFont("Arial", 40, bold=True)
_f_unit = pygame.font.SysFont("Arial", 16)
GA0, GA1 = math.radians(135), math.radians(135 + 270)   # gauge yay araligi

# -------------------- Temalar (skin) --------------------
THEMES = [
    {"name": "KIRMIZI", "bg": (12, 13, 17), "accent": (255, 65, 55),
     "car": (190, 30, 35), "trim": (120, 18, 22), "gtext": (250, 215, 215)},
    {"name": "NARDO",   "bg": (26, 27, 30), "accent": (235, 235, 240),
     "car": (150, 152, 158), "trim": (90, 92, 98), "gtext": (245, 246, 250)},
    {"name": "MAVI",    "bg": (9, 13, 22), "accent": (80, 150, 255),
     "car": (30, 90, 200), "trim": (20, 50, 120), "gtext": (210, 228, 255)},
    {"name": "YESIL",   "bg": (9, 18, 12), "accent": (80, 220, 120),
     "car": (40, 150, 70), "trim": (20, 80, 40), "gtext": (215, 255, 225)},
]
THEME = THEMES[0]


def draw_gauge(cx, cy, R, frac, label, value_text, unit, ticks, redline_frac=None):
    frac = max(0.0, min(1.0, frac))
    pygame.draw.circle(screen, (22, 24, 30), (cx, cy), R)
    pygame.draw.circle(screen, (60, 62, 72), (cx, cy), R, 3)
    # redline yayi
    if redline_frac is not None:
        steps = 24
        for i in range(steps):
            f = redline_frac + (1 - redline_frac) * i / steps
            a = GA0 + (GA1 - GA0) * f
            x = cx + math.cos(a) * (R - 7)
            y = cy + math.sin(a) * (R - 7)
            pygame.draw.circle(screen, (200, 50, 50), (int(x), int(y)), 3)
    # tik isaretleri + rakamlar
    for i in range(ticks + 1):
        f = i / ticks
        a = GA0 + (GA1 - GA0) * f
        ca, sa = math.cos(a), math.sin(a)
        red = redline_frac is not None and f >= redline_frac - 1e-6
        col = (230, 70, 70) if red else (190, 192, 200)
        pygame.draw.line(screen, col, (cx + ca * (R - 6), cy + sa * (R - 6)),
                         (cx + ca * (R - 20), cy + sa * (R - 20)), 3 if red else 2)
        num = _f_small.render(str(int(round(i * (ticks_max_for(ticks, label)) / ticks))),
                              True, col)
        screen.blit(num, (cx + ca * (R - 38) - num.get_width() / 2,
                          cy + sa * (R - 38) - num.get_height() / 2))
    # ibre
    a = GA0 + (GA1 - GA0) * frac
    ca, sa = math.cos(a), math.sin(a)
    pygame.draw.line(screen, THEME["accent"], (cx - ca * 14, cy - sa * 14),
                     (cx + ca * (R - 26), cy + sa * (R - 26)), 4)
    pygame.draw.circle(screen, (210, 210, 220), (cx, cy), 8)
    pygame.draw.circle(screen, (40, 40, 48), (cx, cy), 4)
    # yazilar (gobekle cakismayacak sekilde)
    lbl = _f_unit.render(label, True, (150, 152, 160))
    screen.blit(lbl, (cx - lbl.get_width() / 2, cy - R * 0.52))
    val = _f_big.render(value_text, True, THEME["gtext"])
    screen.blit(val, (cx - val.get_width() / 2, cy + R * 0.22))
    un = _f_unit.render(unit, True, (150, 152, 160))
    screen.blit(un, (cx - un.get_width() / 2, cy + R * 0.50))


def ticks_max_for(ticks, label):
    # gauge ust skala degeri (devir x1000 veya hiz)
    return 8 if label == "RPM x1000" else 320


def draw_shift_lights(cx, y, cur_rpm):
    # F1 tarzi vites isigi: yesil -> sari -> kirmizi, redline'da yanip soner
    leds = 12
    start = 4200.0
    blink = (pygame.time.get_ticks() // 80) % 2 == 0
    x0 = cx - (leds - 1) * 18 / 2
    for i in range(leds):
        f = i / (leds - 1)
        thr = start + (REDLINE - start) * f
        on = cur_rpm >= thr
        if i < 6:
            c_on = (60, 220, 70)
        elif i < 9:
            c_on = (245, 205, 40)
        else:
            c_on = (245, 55, 45)
        if cur_rpm >= REDLINE:           # tavanda: hepsi kirmizi flash
            on, c_on = blink, (255, 45, 45)
        col = c_on if on else (42, 44, 52)
        pygame.draw.circle(screen, col, (int(x0 + i * 18), y), 6)


def draw_car(cx, cy, flame_amt, spin, drift, wheel_ang):
    # Yandan RS7 sportback silueti (one sага bakar), arkada egzoz
    t = pygame.time.get_ticks()
    # --- duman (patinaj/drift) ---
    if spin > 0.25 or drift:
        smoke = pygame.Surface((150, 90), pygame.SRCALPHA)
        n = int(4 + 8 * min(1.0, spin + (0.6 if drift else 0)))
        for i in range(n):
            sx = 20 + (i * 17 + t // 40) % 130
            sy = 60 - (i * 9 + t // 30) % 55
            rad = 8 + (i * 5 % 16)
            al = max(20, 120 - sx)
            pygame.draw.circle(smoke, (180, 180, 185, al), (int(sx), int(sy)), rad)
        screen.blit(smoke, (cx - 145, cy - 40))
    # --- govde ---
    body = [(cx - 90, cy + 15), (cx - 96, cy + 2), (cx - 66, cy - 1),
            (cx - 40, cy - 20), (cx + 22, cy - 22), (cx + 62, cy - 3),
            (cx + 94, cy + 1), (cx + 92, cy + 15)]
    pygame.draw.polygon(screen, THEME["car"], body)           # tema rengi
    pygame.draw.polygon(screen, THEME["trim"], body, 2)
    # cam
    win = [(cx - 34, cy - 17), (cx + 16, cy - 18), (cx + 44, cy - 3), (cx - 38, cy - 2)]
    pygame.draw.polygon(screen, (40, 50, 60), win)
    # --- tekerlekler (donen jant) ---
    for wx in (cx - 55, cx + 55):
        pygame.draw.circle(screen, (25, 25, 28), (wx, cy + 15), 17)
        pygame.draw.circle(screen, (90, 92, 100), (wx, cy + 15), 17, 2)
        for k in range(4):
            a = wheel_ang + k * math.pi / 2
            pygame.draw.line(screen, (160, 162, 170), (wx, cy + 15),
                             (wx + math.cos(a) * 13, cy + 15 + math.sin(a) * 13), 2)
    # --- egzoz uclari (arka sol) ---
    ex, ey = cx - 96, cy + 9
    pygame.draw.rect(screen, (40, 40, 45), (ex - 4, ey - 3, 6, 6))
    pygame.draw.rect(screen, (40, 40, 45), (ex - 4, ey + 5, 6, 6))
    # --- alev (BANG) ---
    if flame_amt > 0.05:
        flick = 0.7 + 0.3 * ((t // 30) % 2)
        flen = (18 + flame_amt * 55) * flick
        for ofy in (0, 8):
            pygame.draw.polygon(screen, (255, 140, 30),
                                [(ex - 3, ey + ofy - 4), (ex - flen, ey + ofy + 1), (ex - 3, ey + ofy + 5)])
            pygame.draw.polygon(screen, (255, 230, 90),
                                [(ex - 3, ey + ofy - 2), (ex - flen * 0.55, ey + ofy + 1), (ex - 3, ey + ofy + 4)])


def draw_telemetry(x, y, w, h):
    pygame.draw.rect(screen, (16, 17, 22), (x, y, w, h))
    pygame.draw.rect(screen, (45, 47, 55), (x, y, w, h), 1)
    pygame.draw.line(screen, (38, 40, 48), (x, y + h / 2), (x + w, y + h / 2), 1)

    def plot(data, col):
        if len(data) < 2:
            return
        n = len(data)
        step = w / (w - 1)
        pts = [(x + i * step, y + h - max(0.0, min(1.0, val)) * h)
               for i, val in enumerate(data)]
        pygame.draw.lines(screen, col, False, pts, 2)

    plot(telem_rpm, (255, 80, 70))     # devir
    plot(telem_spd, (90, 180, 255))    # hiz
    plot(telem_g, (110, 220, 120))     # G
    screen.blit(_f_unit.render("RPM", True, (255, 80, 70)), (x + 8, y + 4))
    screen.blit(_f_unit.render("HIZ", True, (90, 180, 255)), (x + 54, y + 4))
    screen.blit(_f_unit.render("G", True, (110, 220, 120)), (x + 100, y + 4))
    screen.blit(_f_unit.render("TELEMETRI", True, (110, 112, 122)), (x + w - 90, y + 4))


def draw_gmeter(cx, cy, R, g_long, g_lat=0.0):
    pygame.draw.circle(screen, (22, 24, 30), (cx, cy), R)
    pygame.draw.circle(screen, (60, 62, 72), (cx, cy), R, 2)
    pygame.draw.circle(screen, (48, 50, 60), (cx, cy), int(R * 0.5), 1)
    pygame.draw.line(screen, (48, 50, 60), (cx - R, cy), (cx + R, cy), 1)
    pygame.draw.line(screen, (48, 50, 60), (cx, cy - R), (cx, cy + R), 1)
    sc = R / 1.2                          # 1.2g tam olcek
    dx = max(-1.2, min(1.2, g_lat)) * sc
    dy = -max(-1.2, min(1.2, g_long)) * sc
    pygame.draw.circle(screen, (120, 220, 255), (int(cx + dx), int(cy + dy)), 6)
    lbl = _f_unit.render(f"{g_long:+.2f}G", True, (150, 152, 160))
    screen.blit(lbl, (cx - lbl.get_width() / 2, cy + R + 4))


# -------------------- Ana dongu --------------------
running = True
prev_thr = 0.0
prev_gear = gear      # vites degisimi tespiti (rev-match blip)
blip = 0.0            # asagi viteste gaz vurma flare
shift_timer = 0.0     # vites degisiminde kisa tork kesme
while running:
    dt = min(clock.tick(60) / 1000.0, 0.05)
    shift = pygame.key.get_mods() & pygame.KMOD_SHIFT
    for e in pygame.event.get():
        if e.type == pygame.QUIT:
            running = False
        if e.type == pygame.KEYDOWN:
            if e.key == pygame.K_SPACE:               # SPACE = calistir/durdur
                if engine_on and not dying:           # sonmeyi baslat
                    engine_on = False
                    dying = True
                    die_t = 0.0
                    dying_rpm0 = max(rpm, IDLE_RPM)
                elif not engine_on and not cranking and not dying:
                    cranking = True                   # marsi baslat
                    crank_t = 0.0
            elif e.key == pygame.K_a:
                auto = not auto                       # A = oto/manuel
            elif e.key == pygame.K_m:
                exhaust_mode = not exhaust_mode       # M = motor/egzoz sesi
            elif e.key == pygame.K_t:
                stage = (stage + 1) % 4               # T = stage (tuning) degistir
            elif e.key == pygame.K_r:
                reverb.set_env(reverb.env + 1)        # R = ortam (akustik) degistir
            elif e.key == pygame.K_d and not driveby:
                driveby = True                        # D = yanindan gecis (drive-by)
                db_x = -240.0
            elif e.key == pygame.K_c:
                theme_idx = (theme_idx + 1) % len(THEMES)  # C = renk temasi
            elif e.key == pygame.K_n:
                gear = 0 if gear != 0 else pick_gear(v)   # N = bos vites
            elif pygame.K_1 <= e.key <= pygame.K_8:
                gear = e.key - pygame.K_0             # vitese tak (N'den de cikar)
                auto = False
            elif not auto and e.key == pygame.K_x and gear < 8:
                gear = max(1, gear + 1)               # X = yukari vites
            elif not auto and e.key == pygame.K_z and gear > 1:
                gear -= 1                             # Z = asagi vites

    keys = pygame.key.get_pressed()
    # YUKARI = kismi gaz (feather edebilirsin), SHIFT+YUKARI = TAM GAZ
    if keys[pygame.K_UP] and shift:
        throttle = 1.0
    elif keys[pygame.K_UP]:
        throttle = min(throttle + 0.05, 0.80)
    else:
        throttle = max(throttle - 0.06, 0.0)
    # ASAGI = fren
    brake = 1.0 if keys[pygame.K_DOWN] else 0.0

    in_gear = gear != 0
    eng_run = engine_on and not cranking and not dying

    # motor isinmasi (calisirken soguk -> sicak)
    if engine_on and not cranking:
        engine_temp = min(1.0, engine_temp + dt / 50.0)
    current_idle = IDLE_RPM + (1.0 - engine_temp) * 320.0   # soguk fast-idle

    # --- ates / devir durumu ---
    if cranking:
        crank_t += dt
        crank_dur = 1.15 if engine_temp < 0.3 else 0.65    # soguk -> uzun mars
        # duzensiz, yavas mars cevirme
        rpm = 230 + 70 * math.sin(crank_t * 26) + np.random.rand() * 45
        rev_rpm = current_idle
        engine_level = 0.55
        if crank_t > crank_dur:                            # motor tutustu
            cranking = False
            engine_on = True
            eng_run = True
            start_flare = 1.0                              # kisa vroom
            pop_burst = 0.5
    elif dying:
        die_t += dt
        rpm = max(0.0, dying_rpm0 * (1.0 - die_t / 1.1))   # devir 0'a duser
        engine_level = max(0.0, 1.0 - die_t / 1.1)
        if die_t >= 1.1:
            dying = False
            rpm = 0.0
            engine_level = 0.0
    elif not engine_on:
        rpm = 0.0
        throttle = 0.0
        engine_level = 0.0
    else:
        engine_level = 1.0
        if in_gear:
            rpm = max(current_idle, min(rpm_from_speed(v, gear), MAX_RPM))
            rev_rpm = rpm                                  # N'e gecince yumusak
        else:
            rpm = rev_rpm                                  # bos vites: serbest devir

    # --- otomatik vites (sadece calisirken & viteste) ---
    if eng_run and auto and in_gear and shift_timer <= 0:
        if gear < 8 and rpm > 6700 and throttle > 0.05:
            gear += 1
            shift_timer = 0.07
            pop_burst = 1.0                            # yukari vites -> BRAP bang
            bov_burst = max(bov_burst, 0.6)            # vites arasi turbo flutter
        elif gear > 1 and rpm < 2300 and throttle < 0.85:
            gear -= 1
        elif gear > 1 and throttle > 0.92 and rpm < 3200:
            gear -= 1                                  # kickdown
    shift_timer = max(0.0, shift_timer - dt)

    # --- egzoz patlamasi & blow-off tetikleyicileri ---
    if eng_run:
        if prev_thr > 0.45 and throttle < 0.2 and rpm > 2600:
            pop_burst = 1.0                            # gaz birakma -> pat pat
        if prev_thr > 0.55 and throttle < 0.3 and rpm > 2400:
            bov_burst = 1.0                            # boost altinda lift -> psshh
    # --- asagi vites rev-match: gaz vurma blip + bang ---
    if eng_run and 0 < gear < prev_gear and v > 2.0:
        blip = 1.0
        pop_burst = max(pop_burst, 0.7)            # rev-match bang
        bov_burst = max(bov_burst, 0.35)
    prev_gear = gear

    # rev-limiter sert kesme (gaz tam + devir tavanda)
    limiter = eng_run and rpm >= REDLINE - 30 and throttle > 0.5
    prev_thr = throttle

    # --- launch control (dur + fren + tam gaz -> devri tut, birak -> firla) ---
    launching = eng_run and in_gear and v < 0.6 and brake > 0 and throttle > 0.7
    if launching:
        launch_rpm = min(LAUNCH_RPM, launch_rpm + 7000 * dt)
        rpm = launch_rpm                       # debriyaj kayar, devir tutulur
        limiter = False
        if launch_rpm > LAUNCH_RPM - 250 and np.random.rand() < 0.30:
            pop_burst = max(pop_burst, 0.5)    # anti-lag pat pat
    else:
        launch_rpm = max(IDLE_RPM, rpm)
    if was_launching and not launching and throttle > 0.6 and v < 3:
        launch_boost = 0.9                     # kalkis tutus penceresi
    was_launching = launching
    launch_boost = max(0.0, launch_boost - dt)

    # --- tahrik & fizik ---
    if eng_run and in_gear and not launching:
        eng_tq = throttle * torque(rpm)
        if rpm > IDLE_RPM + 120:
            ebrake = (1.0 - throttle) * EB_TQ * (0.35 + 0.65 * rpm / REDLINE)
            eng_tq -= ebrake
        drive = eng_tq * gear_ratios[gear] * FINAL * EFF / WHEEL_R
        if launch_boost > 0:
            drive *= 1.6                       # launch: ekstra tutus
        # --- traksiyon limiti: tutus asilirsa patinaj ---
        if launch_boost == 0 and drive > GRIP and v > 0.3:
            wheelspin = min(1.0, (drive - GRIP) / GRIP)
            drive = GRIP + (drive - GRIP) * 0.45   # bir kismi yine de tutar
        else:
            wheelspin = 0.0
        if rpm >= REDLINE or shift_timer > 0:
            drive = min(drive, 0.0)   # devir limiti / vites kesintisi
    else:
        drive = 0.0                   # bos vites / motor kapali / launch tutuyor
        wheelspin = 0.0

    drag = CD * v * v + ROLL_V * v + ROLL0
    accel = (drive - drag) / MASS
    if brake > 0:
        accel -= 9.5 * brake          # fren ~0.97g
    # g-metre icin gercek boyuna ivme (durma kelepcesi haric)
    g_long = accel / 9.81
    g_smooth += (g_long - g_smooth) * 0.18
    if throttle == 0 and v < 0.4 and brake == 0 and in_gear:
        accel = -v / dt
    v = max(0.0, v + accel * dt)
    if brake > 0 and v < 0.3:
        v = 0.0

    # --- direksiyon + yanal G + drift ---
    sdir = (1 if keys[pygame.K_RIGHT] else 0) - (1 if keys[pygame.K_LEFT] else 0)
    steer += (sdir - steer) * 0.12
    handbrake = keys[pygame.K_b]
    spd_kmh = v * 3.6
    lat_cap = max(0.0, min(1.0, spd_kmh / 55.0))      # hiz olmadan viraj olmaz
    g_lat = steer * lat_cap * 1.05
    total_g = math.hypot(g_smooth, g_lat)
    drifting = eng_run and (total_g > MU * 0.95 or (handbrake and spd_kmh > 18))
    if handbrake and spd_kmh > 12:
        g_lat *= 1.5                                  # el freni -> arka kayar
        v = max(0.0, v - 7.0 * dt)                    # el freni yavaslatir
        wheelspin = max(wheelspin, 0.7)
    if drifting:
        wheelspin = max(wheelspin, min(1.0, abs(g_lat) * 0.6 + 0.3))
        v = max(0.0, v - abs(g_lat) * 2.2 * dt)       # kaymada hiz kaybi
    v = max(0.0, v - abs(g_lat) * 1.2 * dt)           # viraj surtunmesi
    g_lat_smooth += (g_lat - g_lat_smooth) * 0.20

    # --- devir guncelle ---
    if launching:
        pass                                          # rpm zaten launch_rpm
    elif eng_run and in_gear:
        rpm = max(current_idle, min(rpm_from_speed(v, gear), MAX_RPM))
        if launch_boost > 0:
            rpm = max(rpm, 2600)                      # kalkista devri tut
        if wheelspin > 0:
            rpm = min(MAX_RPM, rpm + wheelspin * 1800)  # patinaj devir flare
    elif eng_run:
        # bos viteste serbest devir dinamigi (free-rev)
        net = throttle * torque(rev_rpm) - REV_FR * (rev_rpm - current_idle)
        if rev_rpm >= REDLINE and throttle > 0.05:
            net = -REV_FR * (rev_rpm - current_idle)  # devir limiti: yakit kes
            pop_burst = max(pop_burst, 0.8)
        rev_rpm = min(MAX_RPM, max(current_idle, rev_rpm + net * REV_K * dt))
        rpm = rev_rpm

    # --- rolanti dalgalanmasi (lope): soguk motor daha kaba ---
    if eng_run and throttle < 0.05 and rpm < current_idle + 220:
        idle_phase += dt
        rough = 1.0 + 1.6 * (1.0 - engine_temp)        # soguk -> daha kaba
        rpm += (13 * math.sin(idle_phase * 6.3) + (np.random.rand() - 0.5) * 8) * rough

    # --- calisma vroom flare (mars sonrasi kisa yukselip oturma) ---
    if start_flare > 0.01:
        start_flare *= 0.90
        rpm += start_flare * 550
    else:
        start_flare = 0.0

    # --- asagi vites gaz vurma (rev-match) flare ---
    if blip > 0.01:
        blip *= 0.82
        rpm = min(MAX_RPM, rpm + blip * 450)
    else:
        blip = 0.0

    flame = max(0.0, flame - dt * 4.0)        # alev parlamasi sonumu

    # --- drive-by: arac yanindan gecer (doppler + pan + mesafe) ---
    if driveby:
        DB_SPD = 72.0                         # ~260 km/h gecis
        db_x += DB_SPD * dt
        dy = 6.0                              # yanal mesafe (m)
        d = math.hypot(db_x, dy)
        vr = DB_SPD * (db_x / d)              # radyal hiz (+ uzaklasir)
        doppler = 343.0 / (343.0 + vr)        # yaklasirken tiz, sonra pes
        db_vol = min(1.0, 11.0 / d)
        pan = max(-1.0, min(1.0, db_x / d))
        db_lg = math.sqrt(0.5 * (1 - pan))
        db_rg = math.sqrt(0.5 * (1 + pan))
        if db_x > 240:
            driveby = False
    if not driveby:
        doppler = db_vol = db_lg = db_rg = 1.0

    # --- mesafe & performans kronometresi ---
    dist += v * dt
    if v < 0.6:
        timing = False
        accel_timer = 0.0
        accel_dist0 = dist
        t100_mark = None
        qmark = None
    elif eng_run:
        timing = True
        accel_timer += dt
        if v * 3.6 >= 100 and t100_mark is None:
            t100_mark = accel_timer
            if best_0_100 is None or t100_mark < best_0_100:
                best_0_100 = t100_mark
        if (dist - accel_dist0) >= QMILE and qmark is None:
            qmark = accel_timer
            qtrap = v * 3.6
            if best_qmile is None or qmark < best_qmile[0]:
                best_qmile = (qmark, qtrap)

    speed = v * 3.6

    # --- HUD: Audi Virtual Cockpit tarzi ---
    THEME = THEMES[theme_idx]
    screen.fill(THEME["bg"])
    over = rpm >= REDLINE

    # vites isigi (ust orta)
    draw_shift_lights(WIDTH / 2, 100, rpm if eng_run else 0)

    # iki analog kadran (devir saati 0-8 x1000, redline 6.8)
    draw_gauge(210, 250, 150, rpm / 8000.0, "RPM x1000",
               f"{int(rpm)}", "1/min", 8, redline_frac=REDLINE / 8000.0)
    draw_gauge(710, 250, 150, speed / 320.0, "HIZ",
               f"{int(speed)}", "km/h", 8)

    # g-metre (orta alt) - boyuna + yanal
    draw_gmeter(WIDTH / 2, 390, 44, g_smooth, g_lat_smooth)

    # yan gorunum arac (sag alt) - donen teker, alev, duman
    wheel_ang += (v / TIRE_CIRC) * dt * 6.0
    draw_car(710, 470, flame, wheelspin, drifting, wheel_ang)

    # telemetri grafigi (alt serit)
    TW = WIDTH - 80
    telem_rpm.append(rpm / MAX_RPM)
    telem_spd.append(speed / 320.0)
    telem_g.append((g_smooth + 1.5) / 3.0)
    for buf in (telem_rpm, telem_spd, telem_g):
        if len(buf) > TW:
            del buf[0]
    draw_telemetry(40, 505, TW, 78)

    # orta panel: vites + durum
    gtxt = "N" if gear == 0 else str(gear)
    gear_col = (90, 220, 90) if eng_run else (110, 110, 120)
    gsurf = pygame.font.SysFont("Arial", 110, bold=True).render(gtxt, True, gear_col)
    screen.blit(gsurf, (WIDTH / 2 - gsurf.get_width() / 2, 175))
    mode = "OTO" if auto else "MANUEL"
    msurf = _f_small.render(mode, True, (160, 162, 172))
    screen.blit(msurf, (WIDTH / 2 - msurf.get_width() / 2, 300))

    # motor durumu rozeti (ust orta)
    if cranking:
        est, ecol = "MARŞ...", (255, 200, 80)
    elif dying:
        est, ecol = "SÖNÜYOR...", (255, 160, 80)
    elif engine_on and engine_temp < 0.5:
        est, ecol = "● SOĞUK — ISINIYOR", (120, 200, 255)
    elif engine_on:
        est, ecol = "● MOTOR CALISIYOR", (90, 220, 90)
    else:
        est, ecol = "○ KAPALI — SPACE", (210, 90, 90)
    es = _f_med.render(est, True, ecol)
    screen.blit(es, (WIDTH / 2 - es.get_width() / 2, 30))

    # gaz/fren/ses durumu (orta)
    snd = "EGZOZ" if exhaust_mode else "MOTOR"
    env_nm = BlockReverb.ENV_NAMES[reverb.env]
    info = _f_small.render(f"SES: {snd}   ORTAM: {env_nm}   TEMA: {THEME['name']}", True, (170, 172, 182))
    screen.blit(info, (WIDTH / 2 - info.get_width() / 2, 62))
    # stage rozeti (sol ust)
    scol = [(160, 200, 160), (255, 210, 90), (255, 150, 60), (255, 70, 60)][stage]
    st_txt = f"{STAGE_NAMES[stage]} • ~{STAGE_HP[stage]} hp"
    screen.blit(_f_med.render(st_txt, True, scol), (40, 30))
    if flame > 0.15:
        fb = _f_med.render("🔥 BANG", True, (255, int(120 + 100 * flame), 40))
        screen.blit(fb, (WIDTH / 2 - fb.get_width() / 2, 128))
    if drifting:
        dr = _f_med.render("DRIFT!", True, (255, 90, 200))
        screen.blit(dr, (WIDTH / 2 - dr.get_width() / 2, 152))
    elif wheelspin > 0.05:
        ws = _f_med.render("PATINAJ!", True, (255, 160, 40))
        screen.blit(ws, (WIDTH / 2 - ws.get_width() / 2, 152))
    if launching:
        lc = _f_med.render("◉ LAUNCH CONTROL", True, (255, 210, 70))
        screen.blit(lc, (WIDTH / 2 - lc.get_width() / 2, 128))
    if driveby:
        db = _f_med.render("DRIVE-BY ►►", True, (120, 220, 255))
        screen.blit(db, (WIDTH / 2 - db.get_width() / 2, 128))

    # performans paneli (alt)
    py = 430
    if timing:
        tcur = t100_mark if t100_mark else accel_timer
        screen.blit(_f_med.render(f"0-100: {tcur:.2f}s", True, (120, 220, 255)), (40, py))
    b1 = f"EN IYI 0-100: {best_0_100:.2f}s" if best_0_100 else "EN IYI 0-100: --"
    screen.blit(_f_small.render(b1, True, (150, 200, 150)), (40, py + 28))
    if best_qmile:
        screen.blit(_f_small.render(f"1/4 MIL: {best_qmile[0]:.2f}s @ {int(best_qmile[1])} km/h",
                                    True, (150, 200, 150)), (40, py + 50))

    # kontrol ipuclari (alt)
    screen.blit(_f_small.render("SPACE calistir  •  ↑ gaz  •  SHIFT+↑ tam gaz  •  ↓ fren  •  ←/→ direksiyon  •  B el freni",
                                True, (110, 112, 122)), (40, HEIGHT - 46))
    screen.blit(_f_small.render("A oto/manuel • M ses • T stage • R ortam • C tema • D drive-by • N bos • Z/X vites",
                                True, (110, 112, 122)), (40, HEIGHT - 24))
    pygame.display.flip()

stream.stop()
stream.close()
pygame.quit()
