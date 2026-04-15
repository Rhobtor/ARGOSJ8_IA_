# j8_gui: Estado Operativo Actual

## Objetivo del documento

Este documento resume el estado real de `j8_gui` en el repositorio actual, con foco en:

- que puede hacer hoy la GUI
- que partes estan realmente conectadas al stack ROS2 del J8
- que informacion de personas, robots y paths se mueve por topics o servicios
- que limitaciones siguen existiendo
- que es importante probar antes de una demo o prueba con dos robots

La intencion es dejar una referencia operativa y tecnica simple para el experimento actual, centrada en que hace cada parte y como se conecta con el stack ROS2 del J8.

## Resumen ejecutivo

Estado actual de `j8_gui`:

- la GUI puede trabajar con varios robots usando namespace
- la GUI puede visualizar personas detectadas, asignarlas a robots y cambiar su estado
- la GUI puede crear, cargar, guardar y ejecutar paths
- la GUI envia rutas al `path_manager` real y usa transiciones reales del FSM
- la GUI ya no es solo una maqueta visual: varias partes estan conectadas al stack real del J8
- los roles de robot son actualmente solo informativos o esteticos
- las detecciones de personas se publican por namespace y tambien en un topic global comun
- el sistema sigue necesitando validacion en robot real y en escenario de dos robots a la vez

## Arquitectura resumida

### GUI

`j8_gui` contiene:

- vista de mapa y telemetria
- pestaña de mision y path
- pestaña de control
- pestaña de personas
- soporte para seleccionar robot por namespace

### Backend ROS implicado

Las piezas mas relevantes con las que interactua la GUI son:

- `ctl_mission`
- `path_manager`
- topics de telemetria del robot
- topic de personas detectadas convertido a lat/lon

### Idea de uso prevista para este experimento

El uso mas razonable del sistema actual es:

- la GUI presenta informacion del robot y del entorno
- la GUI permite introducir o editar paths y gestionar personas
- la GUI publica o consume topics y servicios necesarios

En otras palabras: la GUI es una capa de operacion y observacion conectada al stack real.

## Lo que se puede hacer hoy desde la GUI

## Como lanzar el J8 con namespace

El launch principal del stack es este:

- `ros2 launch argj801_setup J8_launch.py robot:=true robot_namespace:=ARGJ801`

Ejemplos utiles:

- robot `ARGJ801`: `ros2 launch argj801_setup J8_launch.py robot:=true robot_namespace:=ARGJ801`
- robot `ARGJ802`: `ros2 launch argj801_setup J8_launch.py robot:=true robot_namespace:=ARGJ802`

Argumentos relevantes de ese launch:

- `robot:=true`: arranca el stack en modo robot
- `robot_namespace:=ARGJ801`: pone todos los nodos, topics y servicios bajo ese namespace



### 1. Seleccionar robot por namespace

La GUI puede operar sobre distintos robots, por ejemplo:

- `ARGJ801`
- `ARGJ802`
- `ARGJ803`

Cuando se cambia el robot activo, la GUI reconecta topics y servicios al namespace correspondiente.

Esto permite usar la misma GUI para operar distintos robots del sistema.

### 2. Ver FSM y enviar transiciones

La GUI puede:

- visualizar el modo actual del FSM
- visualizar transiciones disponibles
- solicitar cambios de modo al stack real

Esto incluye el flujo de `PathFollowing` y otros modos ya expuestos por el J8.

### 3. Gestionar paths

La GUI permite:

- anadir waypoints manualmente
- cargar paths guardados
- guardar paths actuales
- insertar puntos intermedios
- densificar por distancia maxima entre waypoints
- enviar el path al backend
- arrancar seguimiento de path
- parar seguimiento
- cancelar ejecucion y limpiar ruta activa

### 4. Gestionar personas detectadas

La GUI permite:

- recibir personas detectadas en formato global lat/lon
- mostrarlas en la tabla y en el mapa
- mantener un registro con ID
- cambiar manualmente el estado
- asignar una persona a un robot
- liberar la asignacion de una persona
- activar o desactivar seguimiento automatico por distancia
- ajustar umbrales de cambio automatico de estado

### 5. Ver personas en mapa

Las personas se muestran en el mapa con colores por estado:

- `detected`: rojo
- `pending`: amarillo
- `rescued`: no se muestra en el mapa

Una persona rescatada se considera completada y desaparece del mapa.

## Flujo de paths real

El flujo de rutas conectado al stack es este:

1. La GUI construye una ruta en coordenadas lat/lon.
2. La GUI la envia mediante el servicio `RobotPath` a `path_manager`.
3. `path_manager` guarda la ruta activa en memoria.
4. La GUI solicita la transicion del FSM para entrar en `PathFollowing`.
5. El estado `PathFollowing` recupera la ruta desde `path_manager` y la sigue.

Esto significa que la ejecucion de rutas no es una simulacion local de la GUI. La GUI esta usando el flujo operativo del sistema.

## Interfaz real para meter paths en el J8

La GUI nueva no manda los paths por un topic propio de GUI. Los manda al stack del J8 por la interfaz de `path_manager`.

### Opcion 1. Servicio usado por la GUI

La interfaz principal que usa `j8_gui` es este servicio namespaced:

- `/<robot_namespace>/receive_ll_path`

Ejemplo:

- `/ARGJ801/receive_ll_path`

Tipo de servicio:

- `path_manager_interfaces/srv/RobotPath`

Estructura del servicio:

- request: `nav_msgs/Path path`
- response: `bool ack`

La GUI construye el `nav_msgs/Path` con:

- `header.frame_id = "wgs84"`
- cada waypoint en `pose.position.x = lon`
- cada waypoint en `pose.position.y = lat`

### Opcion 2. Topic alternativo aceptado por path_manager

Ademas del servicio, `path_manager` tambien escucha este topic namespaced:

- `/<robot_namespace>/ll_path`

Ejemplo:

- `/ARGJ801/ll_path`

Tipo de mensaje:

- `nav_msgs/msg/Path`

Cuando llega un `Path` por ese topic, `path_manager` lo guarda como ruta activa y lo marca internamente con `frame_id = "ll"`.

### Servicios relacionados con paths ya guardados o activos

Otros nombres utiles bajo namespace son:

- `/<robot_namespace>/get_robot_Path`: devuelve la ruta activa guardada en `path_manager`
- `/<robot_namespace>/read_path_file`: carga una ruta desde fichero
- `/<robot_namespace>/write_path_file`: guarda la ruta actual a fichero
- `/<robot_namespace>/get_ll_path`: devuelve la ruta en lat/lon
- `/<robot_namespace>/path_planner`: servicio de planificacion expuesto por `path_manager`

### Recomendacion practica

Si otra herramienta quiere mandar al J8 exactamente el mismo tipo de path que manda la GUI, lo mas alineado con la implementacion actual es usar:

- `/<robot_namespace>/receive_ll_path`

con un request `path_manager_interfaces/srv/RobotPath`.

## Flujo de personas detectadas

### Origen de las personas

Las personas pueden venir de:

- un nodo fake para pruebas
- un nodo externo real de deteccion

La GUI no genera personas por si sola. Consume un topic ROS ya convertido a lat/lon.

### Conversion local a global

El nodo `detected_persons_local_to_latlon` hace lo siguiente:

1. recibe personas en coordenadas locales del robot
2. usa GNSS e IMU del robot
3. convierte esas personas a lat/lon global
4. publica el resultado en:
   - un topic namespaced del robot
   - un topic global compartido

### Topics de salida de personas

Actualmente hay dos salidas utiles:

1. Topic por robot:

- `/<robot_namespace>/detected_persons_latlon`

Ejemplo:

- `/ARGJ801/detected_persons_latlon`

2. Topic global compartido:

- `/detected_persons_latlon_global`

Este topic global sirve para:

- herramientas que no quieran depender de un robot concreto
- nodos de coordinacion multirobot
- visualizacion o registro comun de detecciones

### Identificacion del robot fuente en el topic global

En el topic global, el mensaje se publica con `header.frame_id` con este formato:

- `wgs84/<namespace>`

Ejemplos:

- `wgs84/ARGJ801`
- `wgs84/ARGJ802`

Esto permite saber que robot genero la deteccion sin depender del nombre del topic.

## Respuesta a la duda clave sobre namespaces

### Pregunta

Si un robot detecta a una persona, y otro robot o una herramienta externa no usa namespace, puede verla?

### Respuesta actual

Si se escucha solo el topic namespaced, no.

Si se escucha el topic global `/detected_persons_latlon_global`, si.

Por tanto:

- un consumidor sin namespace puede suscribirse al topic global
- un sistema externo puede ver detecciones de varios robots en un solo flujo
- la misma GUI puede apoyarse en ese topic comun

### Importante

La GUI actual tambien puede escuchar ese topic global compartido ademas del topic namespaced.

Eso significa que las detecciones pueden verse sin depender exclusivamente del namespace del robot actualmente seleccionado.

## Estados de personas

Cada persona en la GUI puede tener:

- `id`
- `lat`
- `lon`
- `status`
- `status_source`
- `assigned_robot`
- `distance_m`

### Estados soportados

- `detected`
- `pending`
- `rescued`

### Como cambia el estado

Hay dos mecanismos:

1. Manual

- el operador lo cambia desde la pestaña de Personas

2. Automatico por distancia

- si una persona esta suficientemente cerca del robot activo, pasa a `pending`
- si esta aun mas cerca, pasa a `rescued`

### Nota importante

Los roles no controlan el estado de las personas.

Cambiar el rol del robot no cambia por si mismo una persona de rojo a amarillo o de amarillo a rojo. Ese cambio depende del operador o de la logica automatica por distancia.

## Roles de robot

### Estado actual de los roles

Los roles son actualmente informativos.

Sirven para:

- mostrar contexto en la GUI
- etiquetar al robot actual con una funcion semantica

No sirven todavia para:

- cambiar el controlador del robot
- cambiar el FSM
- cambiar el color o estado de personas
- cambiar topics o servicios del stack

### Conclusion sobre roles

En esta version, los roles deben considerarse esteticos o informativos. No son una politica de autonomia.

## Opciones actuales de la pestaña de Personas

La pestaña de Personas se ha simplificado para el experimento actual.

Se mantienen las opciones consideradas utiles:

- filtrar por estado
- ver resumen de detectadas, pendientes y rescatadas
- activar o desactivar seguimiento automatico
- ajustar umbrales de cambio de estado
- asignar robot actual
- liberar asignacion
- marcar detectada
- marcar pendiente
- marcar rescatada

Se han quitado acciones que ya no aportaban a este experimento concreto.

## Multi-robot: que esperar realmente

### Lo que si esta preparado

- operar distintos robots por namespace desde la GUI
- usar topics y servicios namespaced del stack
- tener detecciones globales en un topic comun para consumidores externos

### Lo que hay que validar en pruebas

- que no haya cruces inesperados de control entre robots
- que el robot 1 no ejecute un path del robot 2
- que los cambios de FSM vayan al robot correcto
- que la telemetria y las detecciones se entiendan bien en un escenario de dos robots a la vez

## Topics y servicios utiles

Los puntos mas utiles que ya estan operativos son estos:

- `/<robot_namespace>/fsm_mode`: publica el modo actual del FSM
- `/<robot_namespace>/possible_transitions`: publica las transiciones disponibles
- `/<robot_namespace>/detected_persons_latlon`: publica las personas detectadas por un robot concreto en lat/lon
- `/detected_persons_latlon_global`: publica detecciones de todos los robots en un flujo comun
- servicio `RobotPath` hacia `path_manager`: recibe la ruta activa que se quiere seguir

## Servicios utiles para operar por comandos

Si se quiere operar el stack desde terminal, scripts o herramientas externas, estos son los servicios mas utiles. Todos ellos cuelgan del namespace del robot:

- `/<robot_namespace>/...`

Ejemplo con `ARGJ801`:

- `/ARGJ801/...`

### 1. FSM y estado general

- `/<robot_namespace>/change_fsm` o `/<robot_namespace>/change_fsm_mode`: cambiar transicion del FSM
   - tipo: `ctl_mission_interfaces/srv/ChangeMode`
- `/<robot_namespace>/get_fsm_mode`: consultar estado/lifecycle del nodo de mision
   - tipo: `lifecycle_msgs/srv/GetState`
- `/<robot_namespace>/get_possible_transitions`: consultar transiciones posibles del FSM
   - tipo: `ctl_mission_interfaces/srv/GetPossibleTransitions`

Nota importante:

- en `j8_gui/test_gui_j8` el nombre por defecto usado para cambiar FSM es `change_fsm`
- en `ctl_mission` aparece tambien el nombre interno por defecto `change_fsm_mode`
- antes de automatizar conviene confirmar cual esta expuesto realmente en el robot con `ros2 service list`

### 2. Paths y planificacion

- `/<robot_namespace>/receive_ll_path`: inyectar una ruta completa en `path_manager`
   - tipo: `path_manager_interfaces/srv/RobotPath`
- `/<robot_namespace>/get_robot_Path`: recuperar la ruta activa guardada en `path_manager`
   - tipo: `path_manager_interfaces/srv/ReturnRobotPath`
- `/<robot_namespace>/read_path_file`: cargar una ruta desde fichero
   - tipo: `path_manager_interfaces/srv/ReadPathFromFile`
- `/<robot_namespace>/write_path_file`: guardar la ruta actual a fichero
   - tipo: `path_manager_interfaces/srv/WritePathToFile`
- `/<robot_namespace>/get_ll_path`: pedir la ruta actual en lat/lon
   - tipo: `path_manager_interfaces/srv/GetLLPath`
- `/<robot_namespace>/path_planner`: pedir una planificacion de ruta
   - tipo: `path_manager_interfaces/srv/PlanPath`
- `/<robot_namespace>/assist_emergency`: servicio adicional de apoyo a ruta de emergencia
   - tipo: `path_manager_interfaces/srv/AssistEmergency`

### 3. Controladores de seguimiento

Estos servicios son utiles si se quiere cambiar o reconfigurar el controlador desde fuera:

- `/<robot_namespace>/change_controller_type`
   - tipo: `ctl_mission_interfaces/srv/ChangeController`
- `/<robot_namespace>/config_pure_pursuit`
   - tipo: `ctl_mission_interfaces/srv/ConfigPurePursuitCtrl`
- `/<robot_namespace>/config_stanley`
   - tipo: `ctl_mission_interfaces/srv/ConfigStanleyCtrl`
- `/<robot_namespace>/config_dynamic_pure`
   - tipo: `ctl_mission_interfaces/srv/ConfigDynamicPureCtrl`
- `/<robot_namespace>/config_dynamic_la_pure`
   - tipo: `ctl_mission_interfaces/srv/ConfigDynamicLAPureCtrl`
- `/<robot_namespace>/config_regulated_pure`
   - tipo: `ctl_mission_interfaces/srv/ConfigRegulatedPureCtrl`

### 4. Seguridad

- `/<robot_namespace>/enable_security_check`
   - tipo: `std_srvs/srv/SetBool`
- `/<robot_namespace>/get_security_check`
   - tipo: `security_check_interfaces/srv/GetSecurityCheckStatus`
- `/<robot_namespace>/security_password_check`
   - tipo: `security_check_interfaces/srv/PasswordCheck`

### 5. Plataforma del rover

Servicios expuestos por `argj801_ctrl_platform_node`:

- `/<robot_namespace>/emergency_stop`
   - tipo: `argj801_ctl_platform_interfaces/srv/EmergencyStop`
- `/<robot_namespace>/set_velocity`
   - tipo: `argj801_ctl_platform_interfaces/srv/SetVelocity`
- `/<robot_namespace>/get_velocity`
   - tipo: `argj801_ctl_platform_interfaces/srv/GetVelocity`
- `/<robot_namespace>/ping`
   - tipo: `std_srvs/srv/Empty`

Nota:

- `set_velocity`, `get_velocity` y `ping` dependen del modo de plataforma y pueden no estar disponibles en todos los modos de operacion

## Comprobacion rapida por terminal

Antes de automatizar contra estos nombres, merece la pena comprobar en runtime lo que realmente ha levantado el robot:

- listar servicios del robot: `ros2 service list | grep ARGJ801`
- ver el tipo de un servicio: `ros2 service type /ARGJ801/receive_ll_path`
- ver la interfaz completa: `ros2 interface show path_manager_interfaces/srv/RobotPath`
- listar topics del robot: `ros2 topic list | grep ARGJ801`

## Recomendaciones practicas

- Para enviar rutas desde una herramienta externa, usar `/<robot_namespace>/receive_ll_path`.
- Para leer la ruta activa actual, usar `/<robot_namespace>/get_robot_Path`.
- Para cambiar el FSM, comprobar primero si el nombre real expuesto es `change_fsm` o `change_fsm_mode`.
- Para un entorno con dos robots, mantener siempre el namespace en todas las llamadas y no usar nombres absolutos mezclados entre robots.
- Si se quiere construir automatizacion externa, guardar tambien el resultado de `ros2 service list` y `ros2 topic list` de una sesion real para evitar dudas con nombres finales.

## Limitaciones actuales

### 1. Validacion en robot real pendiente

Aunque la GUI compila y varias integraciones son reales, sigue faltando prueba intensiva en robot real.

### 2. Validacion en dos robots pendiente

La coexistencia multirobot debe probarse especificamente.

### 3. Roles no son comportamiento real

Los roles hoy no cambian el comportamiento del robot.

### 4. La GUI principal del launch historico sigue siendo la legacy

El launch principal del stack aun referencia `GUI_pkg` en lugar de `j8_gui`. Esto debe tenerse en cuenta operativamente.

## Recomendacion para las pruebas de las proximas dos semanas

### Objetivo realista

Cerrar una demo o experimento donde:

- se pueda operar uno o dos robots desde GUI
- se puedan ver personas detectadas
- se puedan asignar personas a robots
- se puedan cargar y ejecutar paths
- se pueda compartir el estado del sistema mediante ROS

### Objetivo que no conviene inflar todavia

No conviene vender esta version como una autonomia multirobot completa guiada por roles.

## Checklist de prueba minima

### Robot unico

1. Arrancar stack y GUI.
2. Ver telemetria y mapa.
3. Cargar o crear un path.
4. Enviarlo y ejecutar `PathFollowing`.
5. Parar y cancelar correctamente.
6. Ver detecciones de personas en GUI.
7. Cambiar manualmente el estado de una persona.
8. Asignar una persona al robot.
9. Confirmar que una rescatada desaparece del mapa.

### Dos robots

1. Arrancar dos stacks con namespaces distintos.
2. Confirmar que cada robot mantiene su control y su telemetria.
3. Confirmar que el topic global de personas recibe detecciones de ambos.
4. Confirmar que una herramienta externa o nodo comun puede leer ese topic global.
5. Confirmar que no se mezclan servicios de FSM ni envio de rutas.

## Conclusion

El sistema actual ya permite una operacion bastante completa desde GUI para el experimento planteado:

- gestion de robots por namespace
- envio y ejecucion de paths
- gestion de personas detectadas
- integracion real con backend ROS del J8
- publicacion global de detecciones para consumidores sin namespace

Lo que falta no es tanto una nueva funcionalidad de interfaz como:

- validacion real
- cerrar el flujo operativo de lanzamiento
- comprobar el escenario con dos robots

Para el objetivo actual, esto es suficiente como base tecnica si se acompana con pruebas bien dirigidas.