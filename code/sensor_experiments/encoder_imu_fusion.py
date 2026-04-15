# Encoder + IMU system ID using ACCELERATION FUSION + RLS
# Model: dv/dt = s*U - bc*sign(v) - k*v
# We estimate [s, b, k] using fused acceleration as y.


from pololu_3pi_2040_robot import robot
import time, math


PAYLOAD_G = 0 # we can change payload and test with different values


WHEEL_RADIUS_M = 0.016
ENC_COUNTS_PER_WHEEL_REV = 360
G0 = 9.80665
METERS_PER_COUNT = (2.0 * math.pi * WHEEL_RADIUS_M) / ENC_COUNTS_PER_WHEEL_REV


SAMPLE_HZ = 50
DT_TARGET = 1.0 / SAMPLE_HZ

# Input profile (rest -> accel -> cruise -> coast)
U_REST   = 0.0
U_ACCEL  = 0.65
U_CRUISE = 0.45

T_REST   = 1.0
T_ACCEL  = 3.0
T_CRUISE = 2.0
T_COAST  = 4.0
T_TOTAL  = T_REST + T_ACCEL + T_CRUISE + T_COAST

def duty_profile(t):
	if t < T_REST:
    	return U_REST
	if t < T_REST + T_ACCEL:
    	return U_ACCEL
	if t < T_REST + T_ACCEL + T_CRUISE:
    	return U_CRUISE
	if t < T_TOTAL:
    	return 0.0
	return None


def alpha_from_tau_lpf(tau, dt):
	# standard 1st order LPF: y = a*x + (1-a)*y
	return dt / (tau + dt)

def alpha_from_tau_hpf(tau, dt):
	# standard 1st order HPF: y = a*(y_prev + x - x_prev)
	return tau / (tau + dt)

class LowPass:
	def __init__(self, alpha, x0=0.0):
    	self.alpha = float(alpha)
    	self.x = float(x0)
	def update(self, x_new):
    	a = self.alpha
    	self.x = a * x_new + (1.0 - a) * self.x
    	return self.x

class HighPass:
	def __init__(self, alpha, x0=0.0):
    	self.alpha = float(alpha)
    	self.y = 0.0
    	self.x_prev = float(x0)
	def update(self, x_new):
    	a = self.alpha
    	y_new = a * (self.y + float(x_new) - self.x_prev)
    	self.x_prev = float(x_new)
    	self.y = y_new
    	return y_new


# Velocity LPF: smooth v from encoder counts
TAU_V_LP = 0.15

# Encoder acceleration LPF: smooth the differentiated velocity
TAU_AENC_LP = 0.10

# IMU: mild LPF for noise, then HPF to eliminate residual bias/tilt drift
TAU_AIMU_LP = 0.06
TAU_AIMU_HP = 0.80

# Optional gain on the IMU high-pass contribution
IMU_HP_GAIN = 1.0

# RLS for dv/dt = s*U - bc*sign(v) - k*v
class RlsEstimator:
	def __init__(self, lam=0.998, p0=80.0):
    	self.lam = float(lam)
  	 
    	self.theta = [3.0, 0.3, 2.0]  
    	self.P = [
        	[p0, 0.0, 0.0],
        	[0.0, p0, 0.0],
        	[0.0, 0.0, p0],
    	]

	def update(self, U, v, y):
    	sv = 1.0 if v > 0.0 else (-1.0 if v < 0.0 else 0.0)
    	phi = [float(U), -sv, -float(v)]

    	# Pphi
    	Pphi = [0.0, 0.0, 0.0]
    	for i in range(3):
        	Pphi[i] = self.P[i][0]*phi[0] + self.P[i][1]*phi[1] + self.P[i][2]*phi[2]

    	denom = self.lam + (phi[0]*Pphi[0] + phi[1]*Pphi[1] + phi[2]*Pphi[2])
    	K = [Pphi[0]/denom, Pphi[1]/denom, Pphi[2]/denom]

    	y_hat = self.theta[0]*phi[0] + self.theta[1]*phi[1] + self.theta[2]*phi[2]
    	err = y - y_hat

    	self.theta[0] += K[0]*err
    	self.theta[1] += K[1]*err
    	self.theta[2] += K[2]*err

    	# phi^T P
    	phiTP = [0.0, 0.0, 0.0]
    	for j in range(3):
        	phiTP[j] = phi[0]*self.P[0][j] + phi[1]*self.P[1][j] + phi[2]*self.P[2][j]

    	newP = [[0.0]*3 for _ in range(3)]
    	for i in range(3):
        	for j in range(3):
            	newP[i][j] = (self.P[i][j] - K[i]*phiTP[j]) / self.lam
    	self.P = newP


def wait_next_tick(last_ms):
	target_ms = int(DT_TARGET * 1000)
	while time.ticks_diff(time.ticks_ms(), last_ms) < target_ms:
    	pass
	now_ms = time.ticks_ms()
	dt_s = time.ticks_diff(now_ms, last_ms) / 1000.0
	return now_ms, dt_s

# IMU setup
USE_IMU = True
IMU_AXIS = "ax"   
IMU_SIGN = +1.0   

def read_forward_accel_mps2(imu):
	imu.read()
	ax, ay, az = imu.acc.last_reading_g
	a_g = ax if IMU_AXIS == "ax" else ay
	return IMU_SIGN * a_g * G0

def calibrate_imu_bias_mps2(imu, seconds=1.0):
	# Robot must be still for this calibration
	n = int(seconds * SAMPLE_HZ)
	s = 0.0
	last_ms = time.ticks_ms()
	for _ in range(n):
    	s += read_forward_accel_mps2(imu)
    	last_ms, _ = wait_next_tick(last_ms)
	return s / max(1, n)

# --- File naming ---
def next_run_index(payload_g):
	for i in range(1, 100):
    	fn = "p{:02d}_rls_fused_{:02d}.csv".format(payload_g, i)
    	try:
        	open(fn, "r").close()
    	except OSError:
        	return i
	return 99


def run_sysid(payload_g):
	display = robot.Display()
	motors = robot.Motors()
	encoders = robot.Encoders()

	imu = None
	a_bias = 0.0
	if USE_IMU:
    	imu = robot.IMU()
    	imu.reset()
    	imu.enable_default()

	run_id = next_run_index(payload_g)
	filename = "p{:02d}_rls_fused_{:02d}.csv".format(payload_g, run_id)

	# Filters
	v_lpf = LowPass(alpha_from_tau_lpf(TAU_V_LP, DT_TARGET), 0.0)
	aenc_lpf = LowPass(alpha_from_tau_lpf(TAU_AENC_LP, DT_TARGET), 0.0)

	aimu_lpf = LowPass(alpha_from_tau_lpf(TAU_AIMU_LP, DT_TARGET), 0.0)
	aimu_hpf = HighPass(alpha_from_tau_hpf(TAU_AIMU_HP, DT_TARGET), 0.0)

	est = RlsEstimator(lam=0.998, p0=80.0)

	f = open(filename, "w")
f.write("t,U,cmd,dist_m,v_mps,v_filt_mps,a_enc_raw,a_enc_lp,a_imu_raw,a_imu_lp,a_imu_hp,a_fused,s_hat,bc_hat,k_h
at,upd\n")

	display.fill(0)
	display.text("SYSID RLS", 0, 0)
	display.text("accel fusion", 0, 10)
	display.show()

	if USE_IMU:
    	time.sleep(0.4)
    	a_bias = calibrate_imu_bias_mps2(imu, seconds=1.0)

	display.fill(0)
	display.text("GO", 0, 0)
	display.text(filename, 0, 10)
	display.show()
	time.sleep(0.8)

	start_ms = time.ticks_ms()
	last_ms = start_ms
	last_vf = 0.0
	max_speed = motors.MAX_SPEED
	dist_m = 0.0

	while True:
    	t = time.ticks_diff(time.ticks_ms(), start_ms) / 1000.0
    	U = duty_profile(t)
    	if U is None:
        	break

    	cmd = int(U * max_speed)
    	motors.set_speeds(cmd, cmd)

    	last_ms, dt = wait_next_tick(last_ms)

  	 
    	dl, dr = encoders.get_counts(reset=True)
    	dF = 0.5 * ((dl * METERS_PER_COUNT) + (dr * METERS_PER_COUNT))
    	dist_m += dF
    	v = dF / dt
    	vf = v_lpf.update(v)

    	a_enc_raw = (vf - last_vf) / dt
    	last_vf = vf
    	a_enc_lp = aenc_lpf.update(a_enc_raw)

    	a_imu_raw = 0.0
    	a_imu_lp = 0.0
    	a_imu_hp = 0.0
    	if USE_IMU:
        	a_imu_raw = read_forward_accel_mps2(imu) - a_bias
        	a_imu_lp = aimu_lpf.update(a_imu_raw)
        	a_imu_hp = aimu_hpf.update(a_imu_lp)


    	a_fused = a_enc_lp + (IMU_HP_GAIN * a_imu_hp)

    	upd = 0
    	if (t > WARMUP_S) and (abs(vf) > MIN_SPEED) and (abs(a_fused) < MAX_ABS_A_FUSED):
        	est.update(U=U, v=vf, y=a_fused)
        	upd = 1

    	s_hat, bc_hat, k_hat = est.theta
    	f.write("{:.3f},{:.3f},{:d},{:.4f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{:.6f},{}\n".format(
        	t, U, cmd, dist_m, v, vf,
        	a_enc_raw, a_enc_lp,
        	a_imu_raw, a_imu_lp, a_imu_hp,
        	a_fused,
        	s_hat, bc_hat, k_hat, upd
    	))

	motors.set_speeds(0, 0)
	f.close()

	display.fill(0)
	display.text("DONE", 0, 0)
	display.text("{:.2f}m".format(dist_m), 0, 10)
	display.show()

	print("Saved:", filename)
	print("Distance:", dist_m, "m")	print("Final [s, bc, k] =", est.theta)

print("=== RUN SYSID RLS (ACCEL FUSION) ===")
print("Payload:", PAYLOAD_G, "g  Total time:", T_TOTAL, "s")
run_sysid(PAYLOAD_G)

