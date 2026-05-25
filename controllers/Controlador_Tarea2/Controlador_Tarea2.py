"""
Laboratorio 2 - Robotica y Sistemas Autonomos 2026-01
ICI 4150 - Navegacion reactiva con filtrado y fusion de sensores en Webots

Controlador para robot e-puck.

Incluye
- Navegacion reactiva con sensores frontales y laterales.
- Lectura de encoders y estimacion de avance, velocidad y odometria.
- Registro de senales crudas, filtradas y fusionadas en CSV.
- Filtro simple de media movil para la senal frontal.
- Filtro de Kalman con etapas de prediccion y correccion.
- Seleccion de modo de decision mediante constante LAB2_MODO(raw, filtered, kalman)
- Salida de consola para visualizar datos.
"""

from controller import Supervisor
from datetime import datetime
import os
import csv
import math

# =============================================================================
# MODO DE DECISION (Modificar ACA!!!!)
# =============================================================================
# Opciones:
#   "crudo" o "raw"
#   "filtrado" o "filtered"
#   "kalman"
LAB2_MODO = "kalman"

# =============================================================================
#  Constantes físicas y de muestreo
# =============================================================================

RADIO_RUEDA = 0.0205        # [m] Radio aproximado de la rueda del e-puck
DISTANCIA_RUEDAS = 0.052    # [m] Distancia entre las ruedas
VELOCIDAD_MAX = 6.28        # [rad/s] Límite de velocidad angular del motor en Webots

TIME_STEP = 64              # [ms] Paso básico de simulación
TS = TIME_STEP / 1000.0     # [s] Tiempo de muestreo
FS = 1.0 / TS               # [Hz] Frecuencia de muestreo


# =============================================================================
#  Constantes de navegación reactiva
# =============================================================================

# Los sensores de proximidad del e-puck entregan valores mayores
# cuando un objeto está más cerca.
UMBRAL_FRONTAL = 105.0          # Obstáculo detectado al frente
UMBRAL_LATERAL = 150.0          # Pared u objeto detectado en los laterales
ZONA_MUERTA_LATERAL = 45.0      # Ignora diferencias laterales pequeñas

# Maniobra de escape cuando el robot queda demasiado cerca de una pared lateral
PASOS_ESCAPE = 40               # Duración de la maniobra de escape
UMBRAL_LATERAL_EXTREMO = 350.0  # Cercanía lateral crítica


# =============================================================================
#  Velocidades y control de centrado
# =============================================================================

VEL_AVANCE = 3.0                # [rad/s] Velocidad base de avance
VEL_GIRO = 2.5                  # [rad/s] Velocidad base para giros evasivos

GANANCIA_CENTRADO = 0.004       # Ganancia proporcional para centrarse en pasillos
CORRECCION_MAX = 0.45           # Límite de corrección para evitar giros bruscos


# =============================================================================
#  Constantes de filtrado y fusión sensorial
# =============================================================================

VENTANA_MEDIA_MOVIL = 5         # Cantidad de muestras para suavizado por media móvil

Q_KALMAN = 0.8                  # Incertidumbre del modelo de predicción
R_KALMAN = 22.0                 # Incertidumbre de la medición del sensor
ESCALA_SENSOR = 120.0           # Conversión empírica: avance [m] -> unidades de sensor


# =============================================================================
#  Etiquetas de acciones para salida en consola
# =============================================================================

ACCION_AVANZAR = "AVANZAR"
ACCION_GIRAR_IZQUIERDA = "GIRAR_IZQUIERDA"
ACCION_GIRAR_DERECHA = "GIRAR_DERECHA"
ACCION_ESCAPE_IZQUIERDA = "ESCAPE_IZQUIERDA"
ACCION_ESCAPE_DERECHA = "ESCAPE_DERECHA"
ACCION_SALIDA_ESCAPE = "SALIDA_ESCAPE"
ACCION_CENTRAR_IZQUIERDA = "CENTRAR_IZQUIERDA"
ACCION_CENTRAR_DERECHA = "CENTRAR_DERECHA"


# =============================================================================
#  Utilidades de consola
# =============================================================================
BOLD = "\033[1m"
RESET = "\033[0m"

def negrita(texto):
    """Devuelve texto con codigo ANSI de negrita para consolas compatibles."""
    return f"{BOLD}{texto}{RESET}"


def separador():
    print("=" * 60)


def limitar(valor, minimo, maximo):
    """Limita un valor al intervalo [minimo, maximo]."""
    return max(minimo, min(maximo, valor))


# =============================================================================
#  Filtros
# =============================================================================

class FiltroMediaMovil:
    """Filtro simple que suaviza la senal promediando las ultimas N muestras."""

    def __init__(self, ventana):
        self.ventana = ventana
        self.buffer = []

    def actualizar(self, valor):
        self.buffer.append(valor)
        if len(self.buffer) > self.ventana:
            self.buffer.pop(0)
        return sum(self.buffer) / len(self.buffer)


class FiltroKalman1D:
    """
    Filtro de Kalman escalar para estimar la proximidad frontal.

    Variable estimada:
        d_hat: proximidad/distancia frontal expresada en unidades del sensor.

    En este laboratorio se piden dos etapas explicitas:
        1) Prediccion: usa el avance calculado con encoders.
        2) Correccion: usa la medicion de los sensores frontales.

    Como el e-puck entrega proximidad y no distancia real en metros, se usa una
    escala empirica para convertir el avance lineal a unidades comparables con
    las lecturas frontales.
    """

    def __init__(self, q, r):
        self.q = q
        self.r = r
        self.d_hat = None
        self.p = 1.0
        self.k = 0.0
        self.d_pred = 0.0
        self.p_pred = 0.0
        

    def inicializado(self):
        return self.d_hat is not None

    def inicializar(self, primera_medicion):
        """Inicializa el estado con la primera lectura real para evitar sesgo."""
        self.d_hat = primera_medicion
        self.p = 1.0
        self.k = 0.0
        self.d_pred = primera_medicion
        self.p_pred = self.p

    def predecir(self, delta_encoder):
        """
        Etapa de prediccion.

        d^-_k = d_hat_{k-1} + delta_encoder
        P^-_k = P_{k-1} + Q
        """
        self.d_pred = self.d_hat + delta_encoder
        self.p_pred = self.p + self.q

    def corregir(self, medicion):
        """
        Etapa de correccion.

        K_k     = P^-_k / (P^-_k + R)
        d_hat_k = d^-_k + K_k (z_k - d^-_k)
        P_k     = (1 - K_k) P^-_k
        """
        self.k = self.p_pred / (self.p_pred + self.r)
        self.d_hat = self.d_pred + self.k * (medicion - self.d_pred)
        self.p = (1.0 - self.k) * self.p_pred
        return self.d_hat

    def actualizar(self, delta_encoder, medicion):
        if not self.inicializado():
            self.inicializar(medicion)
            return self.d_hat

        self.predecir(delta_encoder)
        return self.corregir(medicion)


# =============================================================================
#  Controlador principal
# =============================================================================

class ControladorEpuckLab2:
    """
    Controlador de navegacion reactiva para e-puck.

    Sensores usados:
        ps0: frontal derecho
        ps7: frontal izquierdo
        ps2: lateral derecho
        ps5: lateral izquierdo

    La decision principal puede hacerse con:
        crudo    -> senal frontal sin filtrar
        filtrado -> senal frontal con media movil
        kalman   -> estimacion fusionada con encoders y sensores frontales
    """

    IDX_FRONTAL_DER = 0
    IDX_FRONTAL_IZQ = 7
    IDX_LATERAL_DER = 2
    IDX_LATERAL_IZQ = 5

    MODOS_VALIDOS = {
        "raw": "crudo",
        "crudo": "crudo",
        "filtered": "filtrado",
        "filtrado": "filtrado",
        "kalman": "kalman",
    }

    def __init__(self):
        # Supervisor hereda de Robot y permite acceder a información del mundo.
        self.robot = Supervisor()
        
        # Variables de estado para maniobras de escape y salida.
        self.escape_pasos = 0
        self.escape_direccion = None
        self.salida_pasos = 0
        
        # Motores en modo velocidad continua.
        self.motor_izq = self.robot.getDevice("left wheel motor")
        self.motor_der = self.robot.getDevice("right wheel motor")
        self.motor_izq.setPosition(float("inf"))
        self.motor_der.setPosition(float("inf"))
        self.motor_izq.setVelocity(0.0)
        self.motor_der.setVelocity(0.0)

        # Encoders de rueda.
        self.encoder_izq = self.robot.getDevice("left wheel sensor")
        self.encoder_der = self.robot.getDevice("right wheel sensor")
        self.encoder_izq.enable(TIME_STEP)
        self.encoder_der.enable(TIME_STEP)
        self.encoder_izq_anterior = None
        self.encoder_der_anterior = None

        # Sensores de proximidad ps0 ... ps7.
        self.sensores_ps = []
        for i in range(8):
            sensor = self.robot.getDevice(f"ps{i}")
            sensor.enable(TIME_STEP)
            self.sensores_ps.append(sensor)

        # Filtros.
        self.filtro_media = FiltroMediaMovil(VENTANA_MEDIA_MOVIL)
        self.filtro_kalman = FiltroKalman1D(Q_KALMAN, R_KALMAN)

        # Odometria diferencial.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.muestra = 0
        self.registros = []
        self.modo = self._leer_modo_decision()

        self._imprimir_encabezado()


    def _leer_modo_decision(self):
        modo_ingresado = LAB2_MODO.strip().lower()

        if modo_ingresado not in self.MODOS_VALIDOS:
            print(f"[ADVERTENCIA] Modo inválido: {LAB2_MODO}")
            print("[ADVERTENCIA] Se utilizará: kalman")
            return "kalman"

        return self.MODOS_VALIDOS[modo_ingresado]
    

    def _imprimir_encabezado(self):
        acciones = ", ".join([
            ACCION_AVANZAR,
            ACCION_GIRAR_IZQUIERDA,
            ACCION_GIRAR_DERECHA,
            ACCION_ESCAPE_IZQUIERDA,
            ACCION_ESCAPE_DERECHA,
            ACCION_SALIDA_ESCAPE,
            ACCION_CENTRAR_IZQUIERDA,
            ACCION_CENTRAR_DERECHA,
        ])

        separador()
        print(negrita(" CONTROLADOR LABORATORIO 2 - E-PUCK"))
        separador()
        print(f" Modo de decision      : {self.modo}")
        print(f" Tiempo de muestreo Ts : {TS:.3f} s")
        print(f" Frecuencia fs         : {FS:.2f} Hz")
        print(f" Umbral frontal        : {UMBRAL_FRONTAL:.1f}")
        print(f" Umbral lateral        : {UMBRAL_LATERAL:.1f}")
        print(f" Media movil           : ventana = {VENTANA_MEDIA_MOVIL}")
        print(f" Kalman                : Q = {Q_KALMAN}, R = {R_KALMAN}")
        print(f" Acciones              : {acciones}")
        separador()
        print(negrita(" EJECUCIÓN - E-PUCK"))
        separador()
        print()

    def _leer_encoders_y_odometria(self):
        """
        Lee encoders y calcula:
        - desplazamiento lineal ds,
        - velocidad lineal v,
        - velocidad angular w,
        - pose odometrica aproximada (x, y, theta).
        """
        enc_izq = self.encoder_izq.getValue()
        enc_der = self.encoder_der.getValue()

        if self.encoder_izq_anterior is None:
            self.encoder_izq_anterior = enc_izq
            self.encoder_der_anterior = enc_der
            return enc_izq, enc_der, 0.0, 0.0, 0.0

        d_izq = RADIO_RUEDA * (enc_izq - self.encoder_izq_anterior)
        d_der = RADIO_RUEDA * (enc_der - self.encoder_der_anterior)

        self.encoder_izq_anterior = enc_izq
        self.encoder_der_anterior = enc_der

        ds = (d_izq + d_der) / 2.0
        dtheta = (d_der - d_izq) / DISTANCIA_RUEDAS

        self.theta = (self.theta + dtheta + math.pi) % (2.0 * math.pi) - math.pi
        self.x += ds * math.cos(self.theta)
        self.y += ds * math.sin(self.theta)

        velocidad_lineal = ds / TS
        velocidad_angular = dtheta / TS

        return enc_izq, enc_der, ds, velocidad_lineal, velocidad_angular

    def _obtener_senal_frontal(self, valores_ps):
        """Promedia los sensores frontales izquierdo y derecho."""
        frontal_izq = valores_ps[self.IDX_FRONTAL_IZQ]
        frontal_der = valores_ps[self.IDX_FRONTAL_DER]
        return (frontal_izq + frontal_der) / 2.0

    def _seleccionar_valor_decision(self, frontal_crudo, frontal_filtrado, frontal_kalman):
        """Selecciona la senal que controlara la navegacion."""
        if self.modo == "crudo":
            return frontal_crudo
        if self.modo == "filtrado":
            return frontal_filtrado
        return frontal_kalman

    def _decidir_movimiento(self, valores_ps, valor_decision):
        """
        Decide las velocidades de las ruedas usando navegación reactiva.

        Prioridad de decisión:
        1. Completar maniobras de escape ya iniciadas.
        2. Alejarse si hay una pared lateral crítica u obstáculo frontal peligroso.
        3. Girar si hay obstáculo frontal.
        4. Corregir la trayectoria si hay pared lateral cercana.
        5. Avanzar recto si el camino está libre.
        """
        frontal_izq = valores_ps[self.IDX_FRONTAL_IZQ]
        frontal_der = valores_ps[self.IDX_FRONTAL_DER]
        lateral_izq = valores_ps[self.IDX_LATERAL_IZQ]
        lateral_der = valores_ps[self.IDX_LATERAL_DER]

        # Continúa la maniobra de escape durante los pasos restantes.
        if self.escape_pasos > 0:
            self.escape_pasos -= 1

            if self.escape_pasos == 0:
                self.salida_pasos = 10

            if self.escape_direccion == "derecha":
                return -0.5, -2.0, ACCION_ESCAPE_DERECHA
            return -2.0, -0.5, ACCION_ESCAPE_IZQUIERDA

        # Después del escape, avanza brevemente para separarse del obstáculo.
        if self.salida_pasos > 0:
            self.salida_pasos -= 1
            return VEL_AVANCE, VEL_AVANCE, ACCION_SALIDA_ESCAPE

        lateral_extremo = (
            lateral_izq > UMBRAL_LATERAL_EXTREMO
            or lateral_der > UMBRAL_LATERAL_EXTREMO
        )

        frontal_peligroso = (
            valor_decision > UMBRAL_FRONTAL
            or frontal_izq > UMBRAL_FRONTAL
            or frontal_der > UMBRAL_FRONTAL
        )

        # Inicia una maniobra de escape si el robot está demasiado cerca de un obstáculo.
        if lateral_extremo or frontal_peligroso:
            self.escape_pasos = PASOS_ESCAPE

            if lateral_izq > lateral_der:
                self.escape_direccion = "derecha"
                return -0.5, -2.0, ACCION_ESCAPE_DERECHA

            self.escape_direccion = "izquierda"
            return -2.0, -0.5, ACCION_ESCAPE_IZQUIERDA

        obstaculo_frontal = valor_decision > UMBRAL_FRONTAL
        lateral_izq_cerca = lateral_izq > UMBRAL_LATERAL
        lateral_der_cerca = lateral_der > UMBRAL_LATERAL

        # Si hay obstáculo frontal, gira hacia el lado más despejado.
        if obstaculo_frontal:
            diferencia_lateral = lateral_izq - lateral_der

            if abs(diferencia_lateral) > ZONA_MUERTA_LATERAL:
                girar_derecha = lateral_izq > lateral_der
            else:
                girar_derecha = frontal_izq > frontal_der

            if girar_derecha:
                return VEL_GIRO, -VEL_GIRO, ACCION_GIRAR_DERECHA
            return -VEL_GIRO, VEL_GIRO, ACCION_GIRAR_IZQUIERDA

        # Si una pared lateral está cerca, ajusta velocidades para centrarse.
        if lateral_izq_cerca or lateral_der_cerca:
            # error > 0: pared más cerca a la izquierda, corregir hacia la derecha.
            error = lateral_izq - lateral_der
            correccion = limitar(
                GANANCIA_CENTRADO * error,
                -CORRECCION_MAX,
                CORRECCION_MAX,
            )

            vel_izq = limitar(VEL_AVANCE + correccion, -VELOCIDAD_MAX, VELOCIDAD_MAX)
            vel_der = limitar(VEL_AVANCE - correccion, -VELOCIDAD_MAX, VELOCIDAD_MAX)

            accion = ACCION_CENTRAR_DERECHA if error > 0 else ACCION_CENTRAR_IZQUIERDA
            return vel_izq, vel_der, accion

        return VEL_AVANCE, VEL_AVANCE, ACCION_AVANZAR

    def _imprimir_estado(self, tiempo, frontal_crudo, frontal_filtrado, frontal_kalman,
                         valores_ps, velocidad_lineal, velocidad_angular, accion):
        """Imprime un bloque ordenado con el estado actual del robot."""
        print(f"■ [{negrita('Tiempo')}: {tiempo:.2f}s] [{negrita('Muestra')}: {self.muestra}]")
        print(
            f"   » {negrita('Frontal')}: "
            f"raw={frontal_crudo:.2f}  filt={frontal_filtrado:.2f}  "
            f"kal={frontal_kalman:.2f}  K={self.filtro_kalman.k:.3f}"
        )
        print(
            f"   » {negrita('Lateral')}: "
            f"izq={valores_ps[self.IDX_LATERAL_IZQ]:.2f}  "
            f"der={valores_ps[self.IDX_LATERAL_DER]:.2f}"
        )
        print(
            f"   » {negrita('Odometria')}: "
            f"x={self.x:.3f}  y={self.y:.3f}  th={self.theta:.3f}"
        )
        print(
            f"   » {negrita('Velocidad')}: "
            f"v={velocidad_lineal:.3f}m/s  w={velocidad_angular:.3f}rad/s"
        )
        print(f"   » {negrita('Movimiento')}: {negrita(accion)}")
        print(" ")

    def ejecutar(self):
        """Bucle principal de Webots."""
        while self.robot.step(TIME_STEP) != -1:
            self.muestra += 1
            tiempo = self.muestra * TS

            # 1. Lectura de sensores de proximidad.
            valores_ps = [sensor.getValue() for sensor in self.sensores_ps]

            # 2. Lectura de encoders, avance, velocidades y odometria.
            enc_izq, enc_der, ds, velocidad_lineal, velocidad_angular = self._leer_encoders_y_odometria()

            # 3. Senal frontal cruda.
            frontal_crudo = self._obtener_senal_frontal(valores_ps)

            # 4. Filtro simple: media movil.
            frontal_filtrado = self.filtro_media.actualizar(frontal_crudo)

            # 5. Fusion sensorial: Kalman con prediccion por encoders y correccion por sensores.
            delta_encoder = ds * ESCALA_SENSOR
            frontal_kalman = self.filtro_kalman.actualizar(delta_encoder, frontal_crudo)

            # 6. Seleccion del modo de decision para comparar crudo/filtrado/kalman.
            valor_decision = self._seleccionar_valor_decision(
                frontal_crudo,
                frontal_filtrado,
                frontal_kalman,
            )

            # 7. Navegacion reactiva.
            vel_izq, vel_der, accion = self._decidir_movimiento(valores_ps, valor_decision)
            self.motor_izq.setVelocity(vel_izq)
            self.motor_der.setVelocity(vel_der)

            # 8. Salida periodica de consola. Cambiar 20 por otro valor si se desea mas detalle.
            if self.muestra % 20 == 0:
                self._imprimir_estado(
                    tiempo,
                    frontal_crudo,
                    frontal_filtrado,
                    frontal_kalman,
                    valores_ps,
                    velocidad_lineal,
                    velocidad_angular,
                    accion,
                )

            # 9. Registro de datos para graficar y comparar senales.
            self.registros.append({
                "t_s": round(tiempo, 4),
                "muestra": self.muestra,
                "modo_decision": self.modo,
                "ps0_frontal_der": round(valores_ps[0], 3),
                "ps7_frontal_izq": round(valores_ps[7], 3),
                "ps2_lateral_der": round(valores_ps[2], 3),
                "ps5_lateral_izq": round(valores_ps[5], 3),
                "frontal_crudo": round(frontal_crudo, 3),
                "frontal_filtrado": round(frontal_filtrado, 3),
                "frontal_kalman": round(frontal_kalman, 3),
                "kalman_ganancia": round(self.filtro_kalman.k, 5),
                "kalman_p": round(self.filtro_kalman.p, 5),
                "encoder_izq_rad": round(enc_izq, 5),
                "encoder_der_rad": round(enc_der, 5),
                "avance_ds_m": round(ds, 6),
                "velocidad_lineal_m_s": round(velocidad_lineal, 5),
                "velocidad_angular_rad_s": round(velocidad_angular, 5),
                "odom_x_m": round(self.x, 5),
                "odom_y_m": round(self.y, 5),
                "odom_theta_rad": round(self.theta, 5),
                "valor_decision": round(valor_decision, 3),
                "vel_motor_izq_rad_s": round(vel_izq, 3),
                "vel_motor_der_rad_s": round(vel_der, 3),
                "accion": accion,
            })

        self._guardar_csv()


    def _guardar_csv(self):
        """Guarda las muestras registradas en un CSV identificado por escenario, modo y fecha."""

        if not self.registros:
            return

        ruta_mundo = self.robot.getWorldPath()
        escenario = os.path.splitext(os.path.basename(ruta_mundo))[0]

        fecha_hora = datetime.now().strftime("%d%m%Y-%H%M%S")
        modo_usado = self.modo

        nombre_archivo = f"log_{fecha_hora}_Mundo={escenario}_Modo={modo_usado}.csv"

        ruta = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            nombre_archivo
        )

        with open(ruta, "w", newline="") as archivo:
            escritor = csv.DictWriter(archivo, fieldnames=list(self.registros[0].keys()))
            escritor.writeheader()
            escritor.writerows(self.registros)

        separador()
        print(negrita(" SIMULACION FINALIZADA"))
        print(f" CSV guardado          : {ruta}")
        print(f" Archivo               : {nombre_archivo}")
        print(f" Escenario             : {escenario}")
        print(f" Muestras registradas  : {len(self.registros)}")
        print(f" Modo utilizado        : {self.modo}")
        separador()

if __name__ == "__main__":
    ControladorEpuckLab2().ejecutar()
