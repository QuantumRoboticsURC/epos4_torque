// control_torque: control por torque de un EPOS4 en Cyclic Synchronous Torque.
//
// Sube el torque en rampa hasta el objetivo, lo mantiene y lo baja a cero.
// El setpoint se publica en cada SYNC (100 Hz en epos4_network) desde un
// lazo que no bloquea: StageTargetTorque() sólo escribe un atómico.
//
// Uso:
//   ros2 run epos4_torque control_torque [--iface can0] [--node 5]
//       [--permil 100]     torque en milésimas del torque nominal (0x6076)
//       [--nm 0.05]        ...o en N·m en el eje del motor (usa 0x6076)
//       [--segundos 3]     tiempo sosteniendo el torque
//       [--rampa 0.5]      segundos de subida y de bajada
//       [--vel-max 3000]   rpm; si el motor la supera, el torque va a cero
//       [--dcf ruta/master.dcf]
//
// OJO: sin carga, un torque constante acelera el motor hasta su velocidad
// máxima. --vel-max es la protección; ajústala a tu mecanismo.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <optional>
#include <string>
#include <thread>

#include <ros2units/units.h>

#include "epos4/hardware/Epos4.hpp"

using namespace std::chrono_literals;

namespace
{
volatile std::sig_atomic_t gStop = 0;
void OnSigint(int) {gStop = 1;}

constexpr auto kPeriodo = 10ms;  // igual al sync_period de bus.yml
}  // namespace

int
main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, 0);

  std::string iface = "can0";
  std::string dcf = DEFAULT_DCF;
  int nodeId = DEFAULT_NODE;
  int permil = 100;
  std::optional<double> nm;
  double segundos = 3.0;
  double rampa = 0.5;
  int velMax = 3000;

  for (int i = 1; i + 1 < argc; i += 2) {
    const char * k = argv[i];
    const char * v = argv[i + 1];
    if (!std::strcmp(k, "--iface")) {iface = v;}
    else if (!std::strcmp(k, "--dcf")) {dcf = v;}
    else if (!std::strcmp(k, "--node")) {nodeId = std::atoi(v);}
    else if (!std::strcmp(k, "--permil")) {permil = std::atoi(v);}
    else if (!std::strcmp(k, "--nm")) {nm = std::atof(v);}
    else if (!std::strcmp(k, "--segundos")) {segundos = std::atof(v);}
    else if (!std::strcmp(k, "--rampa")) {rampa = std::max(0.0, std::atof(v));}
    else if (!std::strcmp(k, "--vel-max")) {velMax = std::abs(std::atoi(v));}
    else {
      std::fprintf(stderr, "opción desconocida: %s\n", k);
      return 2;
    }
  }
  std::signal(SIGINT, OnSigint);

  printf("bus %s, DCF %s, nodo %d\n", iface.c_str(), dcf.c_str(), nodeId);

  epos4::CanBus bus{{iface, dcf, 1}};
  epos4::Epos4 motor{bus, static_cast<std::uint8_t>(nodeId)};
  bus.Start();

  auto salir = [&](int codigo) {
      if (motor.IsCyclicModeActive()) {motor.ExitCyclicMode();}
      printf("deshabilitando... %s\n", motor.Disable() ? "ok" : "falló");
      bus.Stop();
      return codigo;
    };

  printf("esperando al EPOS4... ");
  if (!motor.WaitUntilReady()) {
    printf("no responde (¿cableado, bitrate, node-ID?)\n");
    bus.Stop();
    return 1;
  }
  printf("listo\n");

  for (int i = 0; i < 40 && !motor.IsPdoActive(); ++i) {
    std::this_thread::sleep_for(50ms);
  }
  if (!motor.IsPdoActive()) {
    printf("sin PDO: el modo cíclico los necesita (revisa el DCF)\n");
    bus.Stop();
    return 1;
  }

  if (motor.IsFaulted()) {
    printf("falla en el drive: %s\nlimpiando... %s\n",
      motor.DescribeLastError().c_str(), motor.ClearFault() ? "ok" : "sigue en falla");
  }

  // Todo torque del modo cíclico es milésimas de este valor: sin él, --permil
  // no tiene significado físico y el drive haría quién sabe qué.
  auto & torqueNominal = motor.GetMotorRatedTorque().Refresh();
  if (auto ec = torqueNominal.GetStatus()) {
    printf("no se pudo leer el torque nominal (0x6076): %s\n", ec.message().c_str());
    bus.Stop();
    return 1;
  }
  const std::uint32_t nominal = torqueNominal.GetValue();
  printf("torque nominal del motor: %.4f N·m\n", nominal / 1e6);  // 0x6076 en uN·m
  if (nominal == 0) {
    printf("el torque nominal es 0: faltan los datos del motor (corriente nominal y\n"
      "constante de torque, MotorConfigs). Configúralos antes; ver info_motor.\n");
    bus.Stop();
    return 1;
  }

  // El objetivo siempre se lleva en milésimas; --nm se convierte una vez.
  if (nm) {
    const auto pm = motor.TorqueToPerThousand(units::torque::newton_meter_t{*nm});
    if (!pm) {
      printf("no se puede convertir %.4f N·m: falta el torque nominal (MotorConfigs)\n", *nm);
      bus.Stop();
      return 1;
    }
    permil = *pm;
  }
  permil = std::clamp(permil, -1000, 1000);  // no más del nominal en este demo
  // En double: int * uint32_t se calcula sin signo y un objetivo negativo
  // salía como ~4.29 N·m.
  printf("objetivo: %d ‰ del nominal (%.4f N·m)\n", permil, permil * (nominal / 1e9));

  printf("habilitando... ");
  if (!motor.Enable()) {
    printf("FALLÓ\n");
    bus.Stop();
    return 1;
  }
  printf("ok\n");

  if (auto ec = motor.EnterCyclicTorqueMode()) {
    printf("no se pudo entrar a CST: %s\n", ec.message().c_str());
    return salir(1);
  }
  printf("en Cyclic Synchronous Torque, Ctrl+C para parar\n");

  const double tSubida = rampa;
  const double tBajada = rampa + segundos;
  const double tFin = tBajada + rampa;

  const auto inicio = std::chrono::steady_clock::now();
  auto siguiente = inicio;
  int ciclo = 0;
  double tParo = -1.0;  // momento en que empezó a bajar antes de tiempo (Ctrl+C)
  double factorParo = 0.0;
  bool sobrevelocidad = false;
  int codigo = 0;

  while (true) {
    const double t = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - inicio).count();

    if (!motor.IsCyclicHealthy()) {
      printf("¡el eje dejó de responder o salió de Operation enabled!\n");
      if (motor.IsFaulted()) {printf("%s\n", motor.DescribeLastError().c_str());}
      codigo = 1;
      break;
    }

    const std::int32_t vel = motor.GetCachedVelocity();
    // Sobrevelocidad: torque a cero YA. Bajarlo en rampa sigue empujando en
    // el mismo sentido, y el motor seguía acelerando más allá del límite.
    if (!sobrevelocidad && std::abs(vel) > velMax) {
      printf("velocidad %d rpm > %d: torque a cero\n", vel, velMax);
      sobrevelocidad = true;
    }
    if (tParo < 0.0 && gStop) {
      printf("parando...\n");
      tParo = t;
    }

    // Perfil trapezoidal: 0 -> objetivo -> 0. Si se pidió parar, baja en rampa
    // desde donde iba.
    double factor;
    if (t < tSubida) {factor = (rampa > 0.0) ? t / rampa : 1.0;}
    else if (t < tBajada) {factor = 1.0;}
    else if (t < tFin) {factor = (tFin - t) / rampa;}
    else {factor = 0.0;}

    if (sobrevelocidad) {
      factor = 0.0;
    } else if (tParo >= 0.0) {
      if (factorParo == 0.0) {factorParo = std::max(factor, 1e-9);}
      const double r = std::max(rampa, 0.1);
      factor = std::min(factor, factorParo * std::max(0.0, 1.0 - (t - tParo) / r));
    }

    const auto setpoint = static_cast<std::int16_t>(std::lround(permil * factor));
    motor.StageTargetTorque(setpoint);

    if (ciclo % 20 == 0) {  // cada 200 ms
      printf("  t=%5.2fs  cmd=%5d ‰  torque=%5d ‰  vel=%6d rpm  pos=%10d qc\n",
        t, setpoint, motor.GetCachedTorque(), vel, motor.GetCachedPosition());
    }

    if (sobrevelocidad) {
      codigo = 1;
      break;
    }
    if (setpoint == 0 && (t >= tFin || (tParo >= 0.0 && factor == 0.0))) {break;}

    ++ciclo;
    siguiente += kPeriodo;
    std::this_thread::sleep_until(siguiente);
  }

  // Torque cero unos ciclos antes de soltar, para que llegue al drive.
  motor.StageTargetTorque(std::int16_t{0});
  std::this_thread::sleep_for(5 * kPeriodo);
  return salir(codigo);
}
