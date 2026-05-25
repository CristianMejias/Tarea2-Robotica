# Laboratorio 2: Navegación reactiva con filtrado y fusión de sensores en Webots

**Asignatura:** Robótica y Sistemas Autónomos - ICI4150-1  
**Profesora:** Sandra Cano  
**Laboratorio:** Nº 2  
**Robot:** e-puck en Webots  

## Integrantes

- Pablo Aguilera
- Joaquín Garrido
- Benjamín Gómez
- Cristian Mejías
- Cristóbal Rubilar

## Tabla de contenidos

1. [Instrucciones de ejecución](#instrucciones-de-ejecución)
2. [Objetivo](#objetivo)
3. [Descripción general de la solución](#descripción-general-de-la-solución)
4. [Robot y sensores utilizados](#robot-y-sensores-utilizados)
5. [Frecuencia de muestreo](#frecuencia-de-muestreo)
6. [Constantes principales del controlador](#constantes-principales-del-controlador)
7. [Señales registradas](#señales-registradas)
8. [Estimación del avance mediante encoders](#estimación-del-avance-mediante-encoders)
9. [Filtro de media móvil](#filtro-de-media-móvil)
10. [Filtro de Kalman](#filtro-de-kalman)
11. [Lógica de navegación reactiva](#lógica-de-navegación-reactiva)
12. [Escenarios de prueba](#escenarios-de-prueba)
13. [Resultados obtenidos](#resultados-obtenidos)
14. [Análisis de señales](#análisis-de-señales)
15. [Gráficos requeridos](#gráficos-requeridos)
16. [Conclusiones](#conclusiones)

## Instrucciones de ejecución

1. Clonar el repositorio:

    ```bash
    git clone https://github.com/CristianMejias/Tarea2-Robotica.git
    ```

2. Ingresar al directorio del proyecto:

    ```bash
    cd Tarea2-Robotica
    ```

3. Abrir Webots.

4. Abrir uno de los mundos disponibles:

    - `scenario1.wbt`
    - `scenario2.wbt`
    - `scenario3.wbt`

5. Verificar que el robot e-puck tenga asignado el controlador:

    ```text
    Controlador_Tarea2
    ```

6. Seleccionar el modo de decisión editando la constante en el controlador:

    ```python
    LAB2_MODO = "kalman"

    # Opciones disponibles
    # - "crudo" o "raw"
    # - "filtrado" o "filtered"
    # - "kalman"
    ```

7. Ejecutar la simulación desde Webots.

8. Observar en consola los datos periódicos generados por el robot.

9. Finalizar la simulación para que el controlador guarde automáticamente el archivo CSV con los resultados.

10. Generar los gráficos a partir de los CSV y agregarlos en la carpeta `resultados/` o `graficos/`.

## Objetivo

Implementar un sistema de navegación reactiva para un robot móvil diferencial e-puck en Webots, utilizando sensores de proximidad y encoders de rueda. El sistema registra señales crudas, aplica un filtro simple de media móvil, realiza fusión sensorial mediante un filtro de Kalman y usa la estimación obtenida para mejorar la toma de decisiones del robot frente a obstáculos.

El robot no sigue una ruta punto a punto ni una meta obligatoria. Su objetivo es desplazarse de forma autónoma, detectar obstáculos, estimar su movimiento y ejecutar acciones reactivas de avance, corrección lateral, giro o escape según las lecturas disponibles.

## Descripción general de la solución

La solución implementada corresponde a un controlador para el robot e-puck. En cada ciclo de simulación, el robot lee sus sensores frontales y laterales, obtiene la posición angular de los encoders de las ruedas, calcula variables de movimiento y registra las señales relevantes en un archivo CSV.

El controlador permite comparar tres modos de decisión:

| Modo | Señal usada para decidir | Comportamiento esperado |
|---|---|---|
| `crudo` o `raw` | Promedio frontal sin filtrar | Mayor sensibilidad a ruido y picos repentinos. |
| `filtrado` o `filtered` | Promedio frontal suavizado con media móvil | Movimiento más estable, con posible retardo. |
| `kalman` | Estimación fusionada mediante Kalman | Mejor equilibrio entre estabilidad y reacción. |

En los CSV entregados para este informe, los tres escenarios fueron registrados usando el modo `kalman`.

## Robot y sensores utilizados

El robot utilizado es el e-puck de Webots, modelado como un robot móvil diferencial con dos ruedas motrices independientes.

| Parámetro | Valor |
|---|---:|
| Radio de rueda | `0.0205 m` |
| Distancia entre ruedas | `0.052 m` |
| Velocidad máxima de rueda | `6.28 rad/s` |

Sensores utilizados en el controlador:

| Sensor | Ubicación | Uso |
|---|---|---|
| `ps0` | Frontal derecho | Detección frontal de obstáculos. |
| `ps7` | Frontal izquierdo | Detección frontal de obstáculos. |
| `ps2` | Lateral derecho | Detección de pared u obstáculo lateral derecho. |
| `ps5` | Lateral izquierdo | Detección de pared u obstáculo lateral izquierdo. |
| `left wheel sensor` | Rueda izquierda | Encoder para estimar desplazamiento. |
| `right wheel sensor` | Rueda derecha | Encoder para estimar desplazamiento. |

Los sensores de proximidad del e-puck entregan valores en unidades arbitrarias. En esta implementación, valores mayores indican mayor cercanía a un obstáculo, por lo que los umbrales de detección se definen sobre valores de proximidad y no directamente sobre distancia métrica.

## Frecuencia de muestreo

Los mundos de Webots usan un paso básico de simulación de:

```python
TIME_STEP = 64  # ms
```

Por lo tanto:

| Parámetro | Valor |
|---|---:|
| Tiempo de muestreo `Ts` | `0.064 s` |
| Frecuencia de muestreo `fs = 1/Ts` | `15.625 Hz` |

Cantidad de muestras registradas en cada experimento entregado:

| Escenario | Archivo CSV | Modo usado | Duración aproximada | Muestras registradas |
|---|---|---|---:|---:|
| Escenario 1 | `Escenario 1.csv` | `kalman` | `978.56 s` (`16.31 min`) | `15290` |
| Escenario 2 | `Escenario 2.csv` | `kalman` | `937.216 s` (`15.62 min`) | `14644` |
| Escenario 3 | `Escenario 3.csv` | `kalman` | `1491.264 s` (`24.85 min`) | `23301` |

## Constantes principales del controlador

| Constante | Valor | Descripción |
|---|---:|---|
| `RADIO_RUEDA` | `0.0205 m` | Radio aproximado de la rueda del e-puck. |
| `DISTANCIA_RUEDAS` | `0.052 m` | Distancia entre ruedas. |
| `VELOCIDAD_MAX` | `6.28 rad/s` | Límite de velocidad angular de los motores. |
| `UMBRAL_FRONTAL` | `105.0` | Umbral para obstáculo frontal. |
| `UMBRAL_LATERAL` | `150.0` | Umbral para pared u obstáculo lateral. |
| `UMBRAL_LATERAL_EXTREMO` | `350.0` | Umbral para cercanía lateral crítica. |
| `VENTANA_MEDIA_MOVIL` | `5` | Número de muestras usado en el filtro de media móvil. |
| `Q_KALMAN` | `0.8` | Incertidumbre del modelo de predicción. |
| `R_KALMAN` | `22.0` | Incertidumbre asociada a la medición del sensor. |
| `ESCALA_SENSOR` | `120.0` | Conversión empírica entre avance lineal y unidades del sensor. |

## Señales registradas

Durante la simulación, el controlador registra los datos en una lista interna y al finalizar guarda un archivo CSV con los resultados.

Los CSV entregados contienen las siguientes columnas:

| Variable | Descripción |
|---|---|
| `t_s` | Tiempo de simulación en segundos. |
| `muestra` | Número de muestra. |
| `modo_decision` | Modo usado para tomar decisiones. |
| `ps0_frontal_der` | Lectura cruda del sensor frontal derecho. |
| `ps7_frontal_izq` | Lectura cruda del sensor frontal izquierdo. |
| `ps2_lateral_der` | Lectura cruda del sensor lateral derecho. |
| `ps5_lateral_izq` | Lectura cruda del sensor lateral izquierdo. |
| `frontal_crudo` | Promedio de los sensores frontales `ps0` y `ps7`. |
| `frontal_filtrado` | Señal frontal luego del filtro de media móvil. |
| `frontal_kalman` | Estimación frontal obtenida mediante el filtro de Kalman. |
| `kalman_ganancia` | Ganancia de Kalman calculada en cada iteración. |
| `kalman_p` | Covarianza estimada del filtro. |
| `encoder_izq_rad` | Lectura angular del encoder izquierdo. |
| `encoder_der_rad` | Lectura angular del encoder derecho. |
| `avance_ds_m` | Avance lineal estimado entre muestras. |
| `velocidad_lineal_m_s` | Velocidad lineal estimada. |
| `velocidad_angular_rad_s` | Velocidad angular estimada. |
| `odom_x_m`, `odom_y_m`, `odom_theta_rad` | Pose odométrica aproximada. |
| `valor_decision` | Señal usada por la lógica reactiva. |
| `vel_motor_izq_rad_s`, `vel_motor_der_rad_s` | Velocidades enviadas a los motores. |
| `accion` | Acción reactiva ejecutada. |

## Estimación del avance mediante encoders

Los encoders entregan el giro angular acumulado de cada rueda. Para convertir el cambio angular a desplazamiento lineal se usa:

```text
s = r * theta
```

Donde:

- `s` es el desplazamiento lineal de la rueda.
- `r` es el radio de la rueda.
- `theta` es el cambio angular medido por el encoder.

En el controlador se calcula el desplazamiento de cada rueda como:

```python
d_izq = RADIO_RUEDA * (enc_izq - encoder_izq_anterior)
d_der = RADIO_RUEDA * (enc_der - encoder_der_anterior)
```

Luego, el avance promedio del robot se estima mediante:

```python
ds = (d_izq + d_der) / 2.0
```

También se estima el cambio angular del robot:

```python
dtheta = (d_der - d_izq) / DISTANCIA_RUEDAS
```

Con esto se actualiza una odometría aproximada:

```python
theta = theta + dtheta
x = x + ds * cos(theta)
y = y + ds * sin(theta)
```

Esta odometría no corresponde a una localización absoluta perfecta, ya que acumula error, pero permite estimar el desplazamiento del robot y usar esa información dentro del filtro de Kalman.

## Filtro de media móvil

Para suavizar la señal frontal se implementa un filtro de media móvil. La señal frontal cruda corresponde al promedio entre los sensores frontales:

```python
frontal_crudo = (ps7 + ps0) / 2.0
```

El filtro guarda las últimas `N` muestras y calcula su promedio:

```python
frontal_filtrado = promedio(ultimas N muestras)
```

En este laboratorio se usa:

```python
VENTANA_MEDIA_MOVIL = 5
```

Con `Ts = 0.064 s`, una ventana de 5 muestras equivale a `0.32 s` de datos. Este filtro reduce variaciones rápidas de la señal, pero puede introducir retardo en la respuesta frente a obstáculos.

## Filtro de Kalman

Se implementó un filtro de Kalman escalar para estimar la proximidad frontal al obstáculo más cercano. La variable estimada es:

```text
d_hat: proximidad frontal estimada en unidades comparables con el sensor
```

La implementación usa dos fuentes de información:

| Etapa | Fuente de información | Descripción |
|---|---|---|
| Predicción | Encoders | Estima cómo cambia la proximidad frontal según el avance del robot. |
| Corrección | Sensores frontales | Ajusta la estimación usando la medición real del entorno. |

Como los sensores de proximidad del e-puck no entregan directamente distancia en metros, el avance lineal `ds` se convierte a unidades comparables con los sensores mediante:

```python
delta_encoder = ds * ESCALA_SENSOR
```

### Etapa de predicción

```text
d_pred = d_hat_anterior + delta_encoder
p_pred = p_anterior + Q
```

En el código:

```python
self.d_pred = self.d_hat + delta_encoder
self.p_pred = self.p + self.q
```

### Etapa de corrección

La corrección usa la medición frontal cruda `z`, calculada desde los sensores frontales:

```text
K = p_pred / (p_pred + R)
d_hat = d_pred + K * (z - d_pred)
p = (1 - K) * p_pred
```

En el código:

```python
self.k = self.p_pred / (self.p_pred + self.r)
self.d_hat = self.d_pred + self.k * (medicion - self.d_pred)
self.p = (1.0 - self.k) * self.p_pred
```

La ganancia de Kalman aumenta cuando la predicción es más incierta y disminuye cuando la medición es considerada más ruidosa. En los registros entregados, la ganancia converge a valores cercanos a `0.173`, lo que indica un equilibrio estable entre predicción por encoders y corrección por sensores.

## Lógica de navegación reactiva

La navegación se basa en reglas reactivas. En cada iteración se leen sensores y encoders, se calculan las señales cruda, filtrada y fusionada, y se elige un `valor_decision` según el modo configurado.

Prioridad de decisión implementada:

1. Si existe una maniobra de escape activa, el robot la completa.
2. Si acaba de escapar, avanza algunos pasos para separarse del obstáculo.
3. Si detecta cercanía lateral crítica u obstáculo frontal peligroso, inicia escape.
4. Si detecta obstáculo frontal, gira hacia el lado más despejado.
5. Si detecta una pared lateral cercana, corrige su trayectoria para centrarse.
6. Si no hay obstáculos relevantes, avanza recto.

Acciones posibles registradas por el controlador:

| Acción | Descripción |
|---|---|
| `AVANZAR` | Avance recto. |
| `GIRAR_IZQUIERDA` | Giro evasivo hacia la izquierda. |
| `GIRAR_DERECHA` | Giro evasivo hacia la derecha. |
| `ESCAPE_IZQUIERDA` | Maniobra de escape hacia la izquierda. |
| `ESCAPE_DERECHA` | Maniobra de escape hacia la derecha. |
| `SALIDA_ESCAPE` | Avance posterior a una maniobra de escape. |
| `CENTRAR_IZQUIERDA` | Corrección lateral hacia la izquierda. |
| `CENTRAR_DERECHA` | Corrección lateral hacia la derecha. |

## Escenarios de prueba

### Escenario 1: entorno simple

Archivo: `scenario1.wbt`

El primer mundo utiliza una arena rectangular de `2 x 2.5 m`. Incluye paredes laterales, obstáculos cúbicos aislados y un marcador visual verde usado como referencia de observación. El robot comienza cerca de la zona inferior del escenario, con orientación inicial hacia el interior del mundo.

Este escenario está pensado para validar el comportamiento base del controlador: avance recto, detección frontal, corrección frente a paredes laterales y activación de maniobras de escape cuando el robot se aproxima demasiado a obstáculos.

### Escenario 2: entorno con pasillos y obstáculos intermedios

Archivo: `scenario2.wbt`

El segundo mundo utiliza una arena de `3.2 x 3.2 m`. El entorno contiene pasillos estrechos, compuertas de giro y varios obstáculos intermedios distribuidos en distintas zonas. La geometría obliga al robot a usar sensores laterales para no avanzar pegado a una pared y a reaccionar ante obstáculos ubicados cerca de las salidas de los pasillos.

Este escenario permite evaluar la capacidad del controlador para mantener estabilidad en espacios reducidos, activar correcciones laterales y evitar colisiones sucesivas.

### Escenario 3: laberinto

Archivo: `scenario3.wbt`

El tercer mundo utiliza una arena de `4 x 4 m`. Incluye un laberinto con muros horizontales y verticales, pasillos estrechos, esquinas, obstáculos pequeños, cilindros y un pilar central. También contiene un marcador verde de referencia visual, pero el robot no tiene una meta obligatoria asociada a ese marcador.

Este escenario es el de mayor dificultad, ya que exige mantener navegación estable durante más tiempo, resolver situaciones de encierro parcial, evitar giros innecesarios y ejecutar maniobras de escape en pasillos o esquinas.

## Resultados obtenidos

Los tres CSV entregados corresponden a ejecuciones con el modo `kalman`. En todos los escenarios, la acción dominante fue `AVANZAR`, lo que indica que el robot pudo desplazarse durante la mayor parte de la simulación sin quedar bloqueado permanentemente.

### Resumen general por escenario

| Escenario | Modo | Muestras | Duración | Acción predominante | Comentario |
|---|---|---:|---:|---|---|
| Escenario 1 | `kalman` | `15290` | `978.56 s` | `AVANZAR` (`74.2%`) | Entorno simple con escapes simétricos hacia ambos lados. |
| Escenario 2 | `kalman` | `14644` | `937.216 s` | `AVANZAR` (`81.3%`) | Mayor proporción de avance; se observan correcciones laterales en pasillos. |
| Escenario 3 | `kalman` | `23301` | `1491.264 s` | `AVANZAR` (`70.7%`) | Laberinto más exigente; aumenta la cantidad de maniobras de escape. |

### Frecuencia de acciones por escenario

| Acción | Escenario 1 | Escenario 2 | Escenario 3 |
|---|---:|---:|---:|
| `AVANZAR` | `11342` (`74.2%`) | `11908` (`81.3%`) | `16467` (`70.7%`) |
| `ESCAPE_DERECHA` | `1517` (`9.9%`) | `902` (`6.2%`) | `2911` (`12.5%`) |
| `ESCAPE_IZQUIERDA` | `1517` (`9.9%`) | `984` (`6.7%`) | `2276` (`9.8%`) |
| `SALIDA_ESCAPE` | `740` (`4.8%`) | `460` (`3.1%`) | `1260` (`5.4%`) |
| `CENTRAR_DERECHA` | `94` (`0.6%`) | `168` (`1.1%`) | `274` (`1.2%`) |
| `CENTRAR_IZQUIERDA` | `80` (`0.5%`) | `222` (`1.5%`) | `113` (`0.5%`) |

### Estadísticas de señales frontales

| Escenario | Señal | Mínimo | Promedio | Máximo | Desviación estándar |
|---|---|---:|---:|---:|---:|
| Escenario 1 | `frontal_crudo` | `58.319` | `67.671` | `139.111` | `3.868` |
| Escenario 1 | `frontal_filtrado` | `62.569` | `67.670` | `125.517` | `2.890` |
| Escenario 1 | `frontal_kalman` | `63.751` | `69.283` | `108.137` | `2.230` |
| Escenario 2 | `frontal_crudo` | `58.319` | `67.615` | `125.076` | `3.905` |
| Escenario 2 | `frontal_filtrado` | `62.569` | `67.615` | `107.095` | `2.886` |
| Escenario 2 | `frontal_kalman` | `64.261` | `69.450` | `93.437` | `2.102` |
| Escenario 3 | `frontal_crudo` | `58.211` | `67.728` | `121.136` | `3.955` |
| Escenario 3 | `frontal_filtrado` | `62.569` | `67.727` | `104.849` | `2.967` |
| Escenario 3 | `frontal_kalman` | `63.943` | `69.264` | `92.476` | `2.290` |

A partir de estas estadísticas, se observa que la media móvil reduce la variabilidad respecto a la señal cruda. El filtro de Kalman reduce aún más la desviación estándar y limita los máximos, entregando una señal de decisión más estable.

## Análisis de señales

### Señales crudas

Las señales crudas corresponden directamente a las lecturas de los sensores de proximidad y encoders. En los sensores frontales, valores más altos indican mayor cercanía al obstáculo. Estas señales presentan variaciones rápidas debido al ruido de los sensores infrarrojos, cambios de orientación del robot o detecciones parciales de bordes y esquinas.

En los tres escenarios, `frontal_crudo` mantiene un promedio cercano a `67.6`, pero alcanza picos más altos cuando el robot se aproxima a obstáculos. El escenario 1 presenta el mayor máximo de la señal cruda (`139.111`), lo que sugiere encuentros frontales más directos o cercanos con objetos del entorno.

### Señal filtrada por media móvil

La señal `frontal_filtrado` suaviza los cambios rápidos de `frontal_crudo`. Esto se refleja en una menor desviación estándar en los tres escenarios. Por ejemplo, en el escenario 1 la desviación estándar baja de `3.868` en la señal cruda a `2.890` en la señal filtrada.

El costo de este suavizado es que la señal puede responder con un pequeño retardo ante cambios bruscos. Esto es esperable, porque la media móvil utiliza muestras anteriores para calcular el valor actual.

### Señal estimada con Kalman

La señal `frontal_kalman` combina la predicción basada en encoders con la corrección obtenida desde los sensores frontales. En los tres registros, esta señal presenta menor variabilidad que `frontal_crudo` y `frontal_filtrado`. Además, sus máximos son menores, lo que muestra que el estimador evita reaccionar excesivamente ante picos aislados de los sensores.

En navegación reactiva, esta estabilidad es importante porque reduce cambios bruscos en la acción del robot. En los registros entregados, el robot se mantuvo avanzando la mayor parte del tiempo, pero activó escapes y correcciones laterales cuando el entorno lo exigió.

## Gráficos requeridos

La visualización de resultados se organizó de acuerdo con el notebook `Definitivo.ipynb`. Para cada escenario se generan únicamente tres gráficos: uno sin filtro, uno con filtro y uno con Kalman. Al final se agrega una comparación general del desplazamiento por paso entre escenarios.

Los escenarios considerados son: `Básico`, `Simple`, `Complejo` y `Escenario 3`.

### Nota sobre odometría y desplazamiento por paso

El desplazamiento por paso `avance_ds_m` no corresponde a una distancia medida directamente por los sensores de proximidad, sino a una estimación odométrica obtenida desde los encoders de las ruedas. En cada iteración se calcula el avance de cada rueda a partir del cambio angular registrado por los encoders y luego se promedia el avance de ambas ruedas:

```python
d_izq = RADIO_RUEDA * (encoder_izq_actual - encoder_izq_anterior)
d_der = RADIO_RUEDA * (encoder_der_actual - encoder_der_anterior)
avance_ds_m = (d_izq + d_der) / 2.0
```

Este valor permite observar cuándo el robot avanza, retrocede, frena o realiza maniobras de escape. Por eso se incluye en los gráficos como referencia del movimiento real estimado por odometría.

### 1. Sin filtro

En esta primera visualización se muestran las señales frontales crudas del robot: `ps7_frontal_izq`, `ps0_frontal_der` y el promedio `frontal_crudo`. En el mismo gráfico se incorpora el desplazamiento por paso `avance_ds_m`, calculado desde los encoders.

El objetivo de este gráfico es observar el comportamiento directo de los sensores sin procesamiento previo y relacionar los picos de proximidad frontal con los cambios de desplazamiento del robot.

#### Escenario Básico

![Básico — Sin filtro](graficos_lab2_solo_solicitado/básico_01_sin_filtro.png)

#### Escenario Simple

![Simple — Sin filtro](graficos_lab2_solo_solicitado/simple_01_sin_filtro.png)

#### Escenario Complejo

![Complejo — Sin filtro](graficos_lab2_solo_solicitado/complejo_01_sin_filtro.png)

#### Escenario 3

![Escenario 3 — Sin filtro](graficos_lab2_solo_solicitado/escenario_3_01_sin_filtro.png)

### 2. Con filtro

En esta segunda visualización se compara la señal sin filtro contra la señal filtrada para cada sensor frontal. Se analiza por separado el sensor frontal izquierdo y el sensor frontal derecho, y se vuelve a incluir el desplazamiento por paso `avance_ds_m`.

El objetivo de este gráfico es comprobar si el filtro reduce la variación de las señales crudas y si esa suavización se refleja en un movimiento más estable del robot.

#### Escenario Básico

![Básico — Con filtro](graficos_lab2_solo_solicitado/básico_02_con_filtro.png)

#### Escenario Simple

![Simple — Con filtro](graficos_lab2_solo_solicitado/simple_02_con_filtro.png)

#### Escenario Complejo

![Complejo — Con filtro](graficos_lab2_solo_solicitado/complejo_02_con_filtro.png)

#### Escenario 3

![Escenario 3 — Con filtro](graficos_lab2_solo_solicitado/escenario_3_02_con_filtro.png)

### 3. Filtro de Kalman

En esta tercera visualización se presenta la medición frontal cruda frente a la estimación entregada por el filtro de Kalman. También se incluye un gráfico de estabilidad de la estimación, donde se observa la variación móvil de la señal y el error absoluto entre medición y estimación. Finalmente, se agrega nuevamente el desplazamiento por paso `avance_ds_m`.

El objetivo es verificar si la estimación de Kalman entrega una señal más estable y útil para la toma de decisiones del robot, especialmente cuando los sensores frontales presentan ruido o cambios bruscos.

#### Escenario Básico

![Básico — Kalman](graficos_lab2_solo_solicitado/básico_03_kalman.png)

#### Escenario Simple

![Simple — Kalman](graficos_lab2_solo_solicitado/simple_03_kalman.png)

#### Escenario Complejo

![Complejo — Kalman](graficos_lab2_solo_solicitado/complejo_03_kalman.png)

#### Escenario 3

![Escenario 3 — Kalman](graficos_lab2_solo_solicitado/escenario_3_03_kalman.png)

### 4. Comparación por escenario del desplazamiento por paso

El último gráfico compara el desplazamiento por paso `avance_ds_m` en todos los escenarios. La finalidad es observar qué escenario exige más maniobras de corrección, escape o retroceso, y cuál mantiene un avance más continuo.

![Comparación por escenario — desplazamiento por paso](graficos_lab2_solo_solicitado/comparacion_por_escenario_desplazamiento_por_paso.png)

En esta comparación, los valores cercanos a cero representan instantes de poco avance o corrección de trayectoria, mientras que las caídas bajo cero indican retrocesos o maniobras de escape. Las diferencias entre escenarios permiten evaluar cómo cambia el comportamiento del robot según la complejidad del entorno.


## Conclusiones

La implementación permitió analizar la navegación reactiva del robot e-puck utilizando tres niveles de procesamiento de señal: lectura sin filtro, lectura filtrada y estimación mediante Kalman. La incorporación del desplazamiento por paso calculado desde encoders permitió relacionar las mediciones de proximidad con el movimiento real estimado por odometría.

En los gráficos sin filtro se observa que las señales frontales crudas permiten detectar rápidamente los obstáculos, pero también presentan fluctuaciones y picos aislados. Esto puede producir respuestas bruscas en el robot, ya que pequeñas variaciones de los sensores pueden transformarse directamente en cambios de movimiento.

Con el filtro aplicado, las señales frontales izquierda y derecha se vuelven más suaves y fáciles de interpretar. Sin embargo, esta mejora tiene una limitación: al depender de valores anteriores, el filtro puede introducir un pequeño retraso en la reacción. Esto es especialmente importante en escenarios con obstáculos cercanos o sucesivos, donde el robot necesita responder con rapidez.

El filtro de Kalman entregó la señal más estable, ya que combina la medición frontal con una estimación basada en el comportamiento previo del sistema. En los datos registrados, la desviación estándar de `frontal_kalman` fue menor que la de `frontal_crudo` en todos los escenarios, lo que indica una reducción clara de la variabilidad de la señal.

| Escenario | Muestras | Duración aproximada | Avance `AVANZAR` | Maniobras de escape | Desv. est. `frontal_crudo` | Desv. est. `frontal_kalman` |
|---|---:|---:|---:|---:|---:|---:|
| Básico | 14.493 | 927,55 s | 79,9 % | 18,9 % | 3,56 | 2,02 |
| Simple | 15.290 | 978,56 s | 74,2 % | 24,7 % | 3,87 | 2,23 |
| Complejo | 14.644 | 937,22 s | 81,3 % | 16,0 % | 3,91 | 2,10 |
| Escenario 3 | 23.301 | 1491,26 s | 70,7 % | 27,7 % | 3,95 | 2,29 |

La comparación por escenario del desplazamiento por paso muestra que el `Escenario 3` fue el más exigente. Presentó la mayor cantidad de muestras, la mayor duración y la mayor proporción de maniobras de escape. Esto indica que el robot debió corregir su movimiento con mayor frecuencia, probablemente por una geometría del entorno más restrictiva.

El escenario `Complejo`, a pesar de su dificultad, mantuvo el mayor porcentaje de acción `AVANZAR`. Esto sugiere que el controlador logró sostener un movimiento relativamente continuo, aunque con correcciones puntuales cuando los sensores detectaban obstáculos laterales o frontales.

En conjunto, los resultados muestran que la navegación mejora cuando no se depende únicamente de las señales crudas de proximidad. El filtro simple ayuda a reducir ruido, pero Kalman entrega una estimación más robusta para tomar decisiones. Además, la odometría basada en encoders resulta fundamental para interpretar el desplazamiento del robot y comparar el comportamiento entre escenarios.
