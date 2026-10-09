// info_motor: imprime todo lo que conviene revisar de un EPOS4 antes de
// moverlo. Sólo lee: nunca habilita el motor ni cambia nada en el drive.
//
// Uso:
//   ros2 run epos4_torque info_motor [--iface can0] [--node 5] [--dcf ruta/master.dcf]

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "epos4/hardware/Epos4.hpp"

using namespace std::chrono_literals;

int
main(int argc, char ** argv)
{
  setvbuf(stdout, nullptr, _IONBF, 0);

  std::string iface = "can0";
  std::string dcf = DEFAULT_DCF;
  int nodeId = DEFAULT_NODE;

  for (int i = 1; i + 1 < argc; i += 2) {
    const char * k = argv[i];
    const char * v = argv[i + 1];
    if (!std::strcmp(k, "--iface")) {iface = v;}
    else if (!std::strcmp(k, "--dcf")) {dcf = v;}
    else if (!std::strcmp(k, "--node")) {nodeId = std::atoi(v);}
    else {
      std::fprintf(stderr, "opción desconocida: %s\n", k);
      return 2;
    }
  }

  printf("bus %s, DCF %s, nodo %d\n", iface.c_str(), dcf.c_str(), nodeId);

  epos4::CanBus bus{{iface, dcf, 1}};
  epos4::Epos4 motor{bus, static_cast<std::uint8_t>(nodeId)};
  bus.Start();

  printf("esperando al EPOS4... ");
  if (!motor.WaitUntilReady()) {
    printf("no responde\n");
    printf("  revisa: cableado y terminación de 120 ohm, que %s esté arriba con\n"
      "  el bitrate correcto (1 Mbit/s), y que el node-ID del drive sea %d\n"
      "  (en EPOS Studio o con los DIP switches).\n", iface.c_str(), nodeId);
    bus.Stop();
    return 1;
  }
  printf("listo\n");

  for (int i = 0; i < 40 && !motor.IsPdoActive(); ++i) {
    std::this_thread::sleep_for(50ms);
  }

  printf("\n== identidad ==\n");
  epos4::signals::DeviceIdentity id;
  if (auto ec = motor.GetIdentity(id)) {
    printf("  no se pudo leer: %s\n", ec.message().c_str());
  } else {
    printf("%s\n", epos4::signals::Describe(id).c_str());
  }

  printf("\n== estado ==\n");
  printf("  estado CiA402 : %s\n",
    epos4::signals::ToString(motor.GetState().Refresh().GetValue()));
  printf("  modo          : %s\n",
    epos4::signals::ToString(motor.GetOperationMode().Refresh().GetValue()));
  printf("  statusword    : 0x%04X\n", motor.GetStatusword().Refresh().GetValue());
  printf("  PDO           : %s\n", motor.IsPdoActive() ? "activos" : "NO (sólo SDO)");

  std::vector<epos4::signals::PdoMismatch> diferencias;
  if (auto ec = motor.CheckPdoMapping(diferencias)) {
    printf("  mapeo PDO     : no revisado (%s)\n", ec.message().c_str());
  } else if (diferencias.empty()) {
    printf("  mapeo PDO     : drive y maestro coinciden\n");
  } else {
    printf("  mapeo PDO     : %zu DIFERENCIAS\n", diferencias.size());
    for (const auto & d : diferencias) {
      printf("    %s\n", epos4::signals::Describe(d).c_str());
    }
  }

  printf("\n== fallas ==\n");
  if (motor.IsFaulted()) {
    printf("  EN FALLA:\n%s\n", motor.DescribeLastError().c_str());
  } else {
    printf("  sin falla activa\n");
  }
  std::vector<std::uint16_t> historial;
  if (!motor.GetErrorHistory(historial) && !historial.empty()) {
    printf("  historial:");
    for (auto e : historial) {printf(" 0x%04X", e);}
    printf("\n");
  }

  printf("\n== potencia ==\n");
  auto & voltaje = motor.GetSupplyVoltage();
  auto & temp = motor.GetPowerStageTemperature();
  auto & tempMax = motor.GetPowerStageTemperatureLimit();
  auto & corriente = motor.GetMotorCurrentAveraged();
  epos4::signals::RefreshAll(voltaje, temp, tempMax, corriente);
  if (!voltaje.GetStatus()) {printf("  alimentación  : %.1f V\n", voltaje.GetValue().value());}
  if (!temp.GetStatus() && !tempMax.GetStatus()) {
    printf("  etapa potencia: %.1f °C (límite %.1f °C)\n",
      temp.GetValue().value(), tempMax.GetValue().value());
  }
  if (!corriente.GetStatus()) {printf("  corriente     : %.3f A\n", corriente.GetValue().value());}

  printf("\n== motor ==\n");
  epos4::configs::MotorConfigs datos;
  if (auto ec = motor.GetConfigurator().Refresh(datos)) {
    printf("  no se pudieron leer los datos: %s\n", ec.message().c_str());
  } else {
    if (datos.motorType) {
      printf("  tipo              : %s\n", epos4::signals::ToString(*datos.motorType));
    }
    if (datos.nominalCurrent) {
      printf("  corriente nominal : %.3f A\n", *datos.nominalCurrent / 1000.0);
    }
    if (datos.outputCurrentLimit) {
      printf("  límite corriente  : %.3f A\n", *datos.outputCurrentLimit / 1000.0);
    }
    if (datos.torqueConstant) {
      printf("  constante torque  : %.2f mN·m/A\n", *datos.torqueConstant / 1000.0);
    }
    if (datos.numberOfPolePairs) {
      printf("  pares de polos    : %u\n", unsigned{*datos.numberOfPolePairs});
    }
  }
  const std::uint32_t nominal = motor.GetMotorRatedTorque().Refresh().GetValue();
  printf("  TORQUE NOMINAL    : %.4f N·m  (0x6076, base de --permil)\n", nominal / 1e6);
  if (nominal == 0) {
    printf("    = 0: faltan los datos del motor; configúralos antes de usar --nm\n");
  }

  printf("\n== movimiento ==\n");
  printf("  posición : %d qc\n", motor.GetPosition().Refresh().GetValue());
  printf("  velocidad: %d rpm\n", motor.GetVelocity().Refresh().GetValue());
  printf("  torque   : %d ‰\n", motor.GetTorque().Refresh().GetValue());

  bus.Stop();
  return 0;
}
