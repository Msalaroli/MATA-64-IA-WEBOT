from controller import Robot

# intervalos de tempo em ms
TIME_STEP = 64

# inicialização do robô
robot = Robot()

# obtém os devices pelo nome (“camera” e “lidar”)
camera = robot.getDevice('camera')
lidar  = robot.getDevice('lidar')

# habilita ambos os sensores
camera.enable(TIME_STEP)
lidar.enable(TIME_STEP)

print(">>> testBoth iniciado: Camera e Lidar habilitados")

# loop principal
while robot.step(TIME_STEP) != -1:
    # 1) leitura do Lidar
    ranges = lidar.getRangeImage()  # lista de floats
    # imprime três primeiros pontos para ver se está lendo
    print(f"Lidar (3 primeiros): {[f'{r:.2f}' for r in ranges[:3]]} m")
    
    # 2) leitura da Camera (apenas para forçar o frame)
    # A saída visual aparece em View→Cameras no Webots UI
    image = camera.getImage()
    width = camera.getWidth()
    height = camera.getHeight()
    # pega a cor do pixel central só pra confirmar que a API funciona
    cx, cy = width//2, height//2
    r = camera.imageGetRed(image, width, cx, cy)
    g = camera.imageGetGreen(image, width, cx, cy)
    b = camera.imageGetBlue(image, width, cx, cy)
    print(f"Camera pixel central RGB: ({r}, {g}, {b})")
    
    # um pequeno espaçamento
    print("—" * 30)
