
# Importo la libreria per il calcolo numerico
import numpy as np
import sys
import os

# Mi permette di aggiungere tutta la cartella alla directory da cui pescare i file
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

# Dalla cartella labauto importo tutte le classi che mi servono all'interno del progetto
from labauto import BaseController
from labauto import PIDController
from labauto import Delay
from labauto import PinocchioRoboticSystem

# Importo la libreria python per la generazione dei grafici
import matplotlib.pyplot as plt
# Libreria pinocchio per il controllo del corretto caricamento dell'URDF
import pinocchio as pin


# Se le precedenti righe di import sono sottolineate in rosso, bisogna andare dalle quattro linee verticali in alto a
# sinistra, andare su File -> Invalidate caches e ripulire la cache
# Questo succede perchè pycharm non riesce a ricostruire l'indice __init__.py e quindi da un errore

# Ho cancellato from scipy.stats import cosine poichè si riferisce ad una distribuzione statistica
# Non è quindi il coseno trigonometrico, che invece si fa: np.cos(q_position)

class controlloreMassaEccentrica(BaseController):

    # Da capire bene
    def __init__(self, Tc:float):
        super().__init__(Tc)

        # Specifico la cartella dalla quale andare a creare il modello del processo
        model_name = 'progetto_08_P2'

        try:
            self.model = pin.buildModelFromUrdf(f'{model_name}/model.urdf')
            print("URDF caricato correttamente!")
        except Exception as e:
            print("Errore nella lettura del file URDF:")
            print(e)

        # Vado a generare tramite la classe
        self.process_model = PinocchioRoboticSystem(st=0.001, model_name=model_name)
        self.process_model.initialize()

        # Definisco l'oggetto che punta alla classe PID con i parametri stimati dalla simulazione Matlab - Simulink
        self.pid = PIDController(Tc=Tc, Kp=741.6527, Ki=1/0.3849, Kd=0.0962)

        # Definisco l'oggetto che punta alla classe Delay
        self.delay = Delay(Tc=Tc, L=0.2)

    # Metodo che inizializza variabili, costanti e classi di oggetti utilizzate successivamente
    def initialize(self):

        # Inizializzo le costanti utili all'interno del modello
        self.mass = 4.0
        self.accofgravity = 9.81
        self.lenght = 0.5
        self.last_y_speed = 0.0
        self.initial_modelling_error = 0.0
        self.progess_gain = 1.0

        # Inizializzo l'oggetto che punta alla classe dei controllori PID
        self.pid.initialize()
        # Inizializzo l'oggetto che punta alla classe del generatore di ritardo
        self.delay.initialize()


    # Definisco un metodo che faccia lo start delle variabili inerenti al metodo CCA
    def starting(self, reference:float, y_speed:float, q_position: float):

        # Devo far partire il ritardo iniziale da un valore che ipotizzo
        y_speed_delay = self.delay.step(self.last_y_speed)
        #u_pid = self.pid.compute_control_action(reference, y_speed_delay)
        u_pid=0.0
        up_ff = self.mass * self.accofgravity * self.lenght * np.cos(q_position)
        self.pid.starting(reference, y_speed, u_pid, up_ff)
        u_torque = u_pid + up_ff

        # Simula un primo passo di sistema
        y_speed = 0.0#self.process_model.step(u_torque * self.process_gain)
        self.last_y_speed = y_speed


    # Definisco un metodo che mi restituisca la computazione dell'azione di controllo
    def compute_control_action(self, reference:float, y_speed:float, q_position:float)->float:

        y_speed_delay = self.delay.step(self.last_y_speed)
        up_ff = self.mass * self.accofgravity * self.lenght * np.cos(q_position)
        u_pid = self.pid.compute_control_action(reference, y_speed_delay, up_ff)# m*g*l*cos(q)
        u_torque = u_pid + up_ff
        y_speed = 0.0#self.process_model.step(u_torque * self.process_gain)
        self.last_y_speed = y_speed

        return u_torque


# Istanzio il controllore
Tc = 0.01  # tempo di campionamento
controller = controlloreMassaEccentrica(Tc)
controller.initialize()

# Durata simulazione in secondi
T_final = 60.0
n_steps = int(T_final / Tc)

# Vettori per salvare i risultati
time = np.zeros(n_steps)
y_speed_array = np.zeros(n_steps)
u_torque_array = np.zeros(n_steps)
reference_array = np.zeros(n_steps)
q_position_array = np.zeros(n_steps)

# Valori delle tre referenze
v1 = 10.0   # rad/s
v2 = 12.5   # rad/s
v3 = 15.0   # rad/s

# Stato iniziale
y_speed = 0.0
q_position = 0.0  # rad

# Primo step con il metodo `starting`
controller.starting(reference=v1, y_speed=y_speed, q_position=q_position)

# Simulazione nel tempo
for i in range(n_steps):
    t = i * Tc

    # Imposto la reference in base all'intervallo
    if t < 20:
        reference = v1
    elif t < 40:
        reference = v2
    elif t < 60:
        reference = v3
    else:
        break

    # Calcolo azione di controllo
    u_torque = controller.compute_control_action(reference, y_speed, q_position)

    # Aggiorno variabili
    y_speed = controller.last_y_speed  # aggiornato internamente
    q_position += y_speed * Tc         # integrazione della velocità

    # Salvataggio dei dati
    time[i] = t
    y_speed_array[i] = y_speed

    u_torque_array[i] = u_torque
    reference_array[i] = reference
    q_position_array[i] = q_position

# Plot dei risultati
plt.figure(figsize=(12, 5))
plt.plot(time, y_speed_array, label='Velocità y_speed')
plt.plot(time, reference_array, 'k--', label='Riferimento')
plt.xlabel('Tempo [s]')
plt.ylabel('Velocità angolare [rad/s]')
plt.title('Risposta in velocità del sistema')
plt.legend()
plt.grid()

plt.figure(figsize=(12, 5))
plt.plot(time, u_torque_array, color='tab:orange', label='Azione di controllo u_torque')
plt.xlabel('Tempo [s]')
plt.ylabel('Coppia [Nm]')
plt.title('Azione di controllo')
plt.legend()
plt.grid()

plt.figure(figsize=(12, 5))
plt.plot(time, q_position_array, color='tab:green', label='Posizione angolare q [rad]')
plt.xlabel('Tempo [s]')
plt.ylabel('Angolo [rad]')
plt.title('Evoluzione della posizione angolare')
plt.legend()
plt.grid()

plt.show()






