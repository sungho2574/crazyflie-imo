// TOGT-Planner -> Crazyflie 궤적 변환기 (오프라인 도구)
//
// FSC-Lab/TOGT-Planner 로 시간최적 궤적을 계획한 뒤, 그 결과를 크플 펌웨어의
// High-Level Commander 가 실행하는 **조각별 다항식 CSV** 로 내보낸다.
//
//   togt_plan <params_dir> <setups.yaml> <track.yaml> <out.csv>
//
// 크플 CSV 포맷 (crazyswarm2 uav_trajectory.Trajectory.loadcsv 가 읽는 33열):
//   duration, x^0..x^7, y^0..y^7, z^0..z^7, yaw^0..yaw^7
//   - 축마다 8계수(7차), 오름차순(c0 + c1 t + ... + c7 t^7)
//   - 시간은 조각별 절대초 (t in [0, duration])
//
// TOGT 쪽 규약 (검증 완료):
//   MincoSnapTrajectory.polys : PiecewisePolynomial<7>  (struct 라 public)
//   piece.getCoeffMat() : 3x8, 행=x/y/z, 열은 **내림차순**(col0=t^7 ... col7=t^0), 절대초
//   piece.getDuration() : 초
//   -> 크플로 가려면 각 행의 열을 뒤집기만 하면 된다(시간 정규화 불필요).
//   yaw 는 TOGT 궤적에 다항식이 없다 -> 상수 0 으로 채운다(기수 고정).
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "drolib/race/race_params.hpp"
#include "drolib/race/race_planner.hpp"
#include "drolib/race/race_track.hpp"

using namespace drolib;

int main(int argc, char** argv) {
  if (argc < 5) {
    std::cerr << "usage: togt_plan <params_dir> <setups.yaml> <track.yaml> "
                 "<out.csv>\n";
    return 2;
  }
  const std::filesystem::path params_dir(argv[1]);
  const std::string setups(argv[2]);
  const std::filesystem::path track_path(argv[3]);
  const std::string out_csv(argv[4]);

  auto params = std::make_shared<RaceParams>(params_dir, setups);
  auto planner = std::make_shared<RacePlanner>(*params);
  auto track = std::make_shared<RaceTrack>(track_path);

  if (!planner->plan(track)) {
    std::cerr << "[togt_plan] planning failed\n";
    return 1;
  }

  // 실기체 적용성 점검용 극값 (최대 속도/각속도/틸트 등)
  std::cout << planner->getExtremum() << std::endl;

  const MincoSnapTrajectory traj = planner->getTrajectory();
  const auto& polys = traj.polys;
  const int n = polys.getPieceNum();

  std::ofstream f(out_csv);
  if (!f) {
    std::cerr << "[togt_plan] cannot open " << out_csv << "\n";
    return 1;
  }
  // 헤더 (crazyswarm figure8.csv 와 동일, 끝에 콤마)
  f << "duration";
  const char* ax[4] = {"x", "y", "z", "yaw"};
  for (int a = 0; a < 4; ++a)
    for (int k = 0; k < 8; ++k) f << "," << ax[a] << "^" << k;
  f << ",\n";
  f << std::setprecision(10);

  for (int i = 0; i < n; ++i) {
    const auto& piece = polys[i];
    const double dur = piece.getDuration();
    const auto C = piece.getCoeffMat();   // 3x8, col0=t^7 ... col7=t^0
    f << dur;
    for (int row = 0; row < 3; ++row)     // x, y, z
      for (int k = 0; k < 8; ++k)         // 오름차순으로 뒤집기: c_k = C(row, 7-k)
        f << "," << C(row, 7 - k);
    for (int k = 0; k < 8; ++k) f << "," << 0.0;   // yaw = 0 (기수 고정)
    f << ",\n";
  }
  f.close();

  std::cout << "[togt_plan] pieces = " << n
            << ", total duration = " << polys.getTotalDuration() << " s\n";
  std::cout << "[togt_plan] saved " << out_csv << "\n";
  return 0;
}
