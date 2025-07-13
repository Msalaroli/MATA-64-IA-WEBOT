# controllers/sensorDataCollector/sensorDataCollector.py

from controller import Robot
import csv, os, math, random

# ------- Parâmetros -------
TIME_STEP   = 64        # ms
MAX_STEPS   = 2000      # número de iterações
DATA_DIR    = "data"    # pasta onde imagens e CSV serão salvos
IMG_PREFIX  = "img"     # prefixo dos arquivos de imagem

# ------- Inicialização -------
robot  = Robot()
timestep = int(robot.getBasicTimeStep())

# Sensores
camera = robot.getDevice('camera')
lidar  = robot.getDevice('lidar')
camera.enable(TIME_STEP)
camera.recognitionEnable(TIME_STEP)
lidar.enable(TIME_STEP)

# Motores (4 rodas)
lf = robot.getDevice('front left wheel')
lr = robot.getDevice('back left wheel')
rf = robot.getDevice('front right wheel')
rr = robot.getDevice('back right wheel')
for m in (lf, lr, rf, rr):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)

# Prepara saída
os.makedirs(DATA_DIR, exist_ok=True)
csvf   = open(os.path.join(DATA_DIR, 'dataset.csv'), 'w', newline='')
writer = csv.writer(csvf)
writer.writerow(['image','dist_obs','ang_obs','ang_target'])

# Pre-calcula campo de visão horizontal
h_fov = lidar.getFov()  # obtém o FOV horizontal do Lidar

step = 0
while robot.step(TIME_STEP) != -1 and step < MAX_STEPS:
    # Navegação: random walk
    if step % (1000 // TIME_STEP) == 0:
        turn  = random.uniform(-1, 1) * 1.5
        speed = 3.0
        left_speed  = speed + turn
        right_speed = speed - turn
        for m in (lf, lr):
            m.setVelocity(left_speed)
        for m in (rf, rr):
            m.setVelocity(right_speed)

    # LiDAR: distância e ângulo do obstáculo mais próximo
    ranges    = lidar.getRangeImage()
    dist_obs  = min(ranges)
    idx_min   = ranges.index(dist_obs)
    total_beams = len(ranges)
    ang_obs   = (idx_min - total_beams/2) * (h_fov/total_beams)

    # Câmera: ângulo ao alvo amarelo
    ang_target = 0.0
    for obj in camera.getRecognitionObjects():
        if obj.get_colors()[0] == 0xFFFF00:
            x, _, z = obj.get_position()
            ang_target = math.atan2(x, z)
            break

    # Salvamento de imagem e CSV
    img_name = f"{IMG_PREFIX}_{step:04d}.png"
    camera.saveImage(os.path.join(DATA_DIR, img_name), quality=90)
    writer.writerow([img_name,
                     f"{dist_obs:.4f}",
                     f"{ang_obs:.4f}",
                     f"{ang_target:.4f}"])

    step += 1

csvf.close()
print("→ Coleta concluída! Veja em controllers/sensorDataCollector/data/")
