import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
from numpy.linalg import inv
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import interp1d

# Lee archivo de datos
df = pd.read_parquet(os.path.join(os.path.dirname(__file__), 'data','trajectory_data.parquet'))

# Crea funciones de interpolación para posición y velocidad
interp_q1 = interp1d(df['time'], df['q1'], kind='linear', fill_value='extrapolate')
interp_q2 = interp1d(df['time'], df['q2'], kind='linear', fill_value='extrapolate')

interp_dq1 = interp1d(df['time'], df['dq1'], kind='linear', fill_value='extrapolate')
interp_dq2 = interp1d(df['time'], df['dq2'], kind='linear', fill_value='extrapolate')

# Ruta al modelo de simulación
xml_path = os.path.join(os.path.dirname(__file__), "assets","5_bar.xml") 

# Tiempo de simulación (en segundos)
simend = 4 

# Configuración de interacción del teclado y mouse
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    if (not button_left) and (not button_middle) and (not button_right):
        return

    width, height = glfw.get_window_size(window)

    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)


# Objetos de Mujoco
model = mj.MjModel.from_xml_path(xml_path)  # Modelo
data = mj.MjData(model)                     # data
cam = mj.MjvCamera()                        # camara 
opt = mj.MjvOption()                        # opciones de visualización


# Inicializa GLFW
glfw.init()
window = glfw.create_window(1200, 900, "Sim", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# Inicializa estructuras de visualización
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# Configura callbacks de mouse y teclado
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Configuración de la cámara
cam.azimuth = 90.11676025390628 ; cam.elevation = -50.03149414062499 ; cam.distance =  2
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])
print_camera_config = 0 

# Controlador
q1_0=df.q1[0]
q2_0=df.q2[0]

ctrl1_values = []
ctrl2_values = []

times,pxs,pys,vxs,vys,axs,ays,eff_xs,eff_ys=[],[],[],[],[],[],[],[],[]


def init_controller(model,data):
    data.qvel[0] = 0
    data.qvel[2] = 0



def controller(model, data):
    # Lee variables de estado
    q1=data.qpos[0]
    q2=data.qpos[2]
    
    dq1 = data.qvel[0]
    dq2 = data.qvel[2]
    
    ddq1 = data.qacc[0]
    ddq2 = data.qacc[2]
    
    # Lee un sensor en el actuador
    pos_sens_x=data.sensordata[0]
    pos_sens_y=data.sensordata[1]

    kp=0.01
    kv=0.2
    #Le permite 2 segundos al robot para llegar a la posición inicial de la trayectoria.
    # Esto le da mayor estabilidad a la simulación ya que parte de la posición inicial (conocida) del modelo
    if(data.time<2):
        data.ctrl[0] = -10*(q1-(q1_0))-0.5*dq1
        data.ctrl[1] = -10*(q2-(q2_0))-0.5*dq2
    else:
        data.ctrl[0] = -kp*(q1-(interp_q1(data.time-2)))-kv*(dq1-interp_dq1(data.time-2))
        data.ctrl[1] = -kp*(q2-(interp_q2(data.time-2)))-kv*(dq2-interp_dq2(data.time-2))
        
        ctrl1_values.append(data.ctrl[0])
        ctrl2_values.append(data.ctrl[1])
        
        times.append(data.time-2)
        
        pxs.append(q1)
        pys.append(q2)

        vxs.append(dq1)
        vys.append(dq2)

        axs.append(ddq1)
        ays.append(ddq2)
        
        eff_xs.append(pos_sens_x)
        eff_ys.append(pos_sens_y)



init_controller(model,data)

mj.set_mjcb_control(controller)

# Corre la simulación
while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)
        
    if (data.time>=simend):
        break

    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    glfw.swap_buffers(window)

    glfw.poll_events()

glfw.terminate()

# Crea subplots
fig, axis = plt.subplots(5, 1, figsize=(8, 12))

time_obj=np.linspace(0,2,20)

# Grafica posiciones
axis[0].plot(times, pxs,'r-', label='q1')
axis[0].plot(times, pys, 'b-',label='q2')
axis[0].plot(time_obj, interp_q1(time_obj),'r*', label='q1_obj')
axis[0].plot(time_obj, interp_q2(time_obj),'b*', label='q1_obj')
axis[0].set_title('Positions')
axis[0].set_xlabel('Time')
axis[0].set_ylabel('Position')
axis[0].legend()

# Grafica velocidades
axis[1].plot(times, vxs, 'r-',label='dq1')
axis[1].plot(times, vys, 'b-',label='dq2')
axis[1].plot(time_obj, interp_dq1(time_obj),'r*', label='dq1_obj')
axis[1].plot(time_obj, interp_dq2(time_obj),'b*', label='dq1_obj')
axis[1].set_title('Velocities')
axis[1].set_xlabel('Time')
axis[1].set_ylabel('Velocity')
axis[1].legend()

# Grafica aceleraciones
axis[2].plot(times, axs, 'r-',label='ddq1')
axis[2].plot(times, ays, 'b-',label='ddq2')
axis[2].set_title('Accelerations')
axis[2].set_xlabel('Time')
axis[2].set_ylabel('Acceleration')
axis[2].legend()

# Grafica posiciones del efector en el espacio de trabajo
axis[3].plot(eff_xs, eff_ys, 'k-')
axis[3].set_title('Effector Position')
axis[3].set_xlabel('x (m)')
axis[3].set_ylabel('y (m)')

# Grafica torque
axis[4].plot(times, ctrl1_values, 'r-',label='Motor 1')
axis[4].plot(times, ctrl2_values, 'b-',label='Motor 2')
axis[4].set_title('Actuator Torque')
axis[4].set_xlabel('Time')
axis[4].set_ylabel('Torque')
axis[4].legend()

# Ajusta el layout de las gráficas
plt.tight_layout()
plt.show()
