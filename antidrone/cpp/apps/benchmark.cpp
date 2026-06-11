// 추적 파이프라인 정량화 벤치마크 (헤드리스, GT 불필요)
//
// 현재 TrackingPipeline이 실제로 사용하는 경로를 프레임별로 집계한다.
// result.target->source(TrackSource)가 매 프레임 어느 단계에서 타깃이
// 나왔는지 라벨해 주므로, 정답 라벨 없이도 다음을 정량화할 수 있다:
//   - 모드 점유율(ByteTrack / CSRT / Kalman 홀드 / 손실)
//   - Kalman 홀드 전환 빈도와 홀드 구간 길이
//   - 선택 타깃의 ID 전환 횟수(연속 추적 중 trackId 변화)
//   - 추적 커버리지(타깃 존재 프레임 비율)와 추적 구간 연속성
//
// 입력은 재현성을 위해 비디오 파일을 권장하며, dt는 영상 FPS로 고정한다.

#include "ptcamera/pipeline.hpp"
#include "ptcamera/settings.hpp"
#include "ptcamera/types.hpp"

#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>

// 이 빌드에 OpenCV contrib 트래커(CSRT/KCF)가 포함됐는지 동일 기준으로 점검.
#if __has_include(<opencv2/tracking.hpp>)
#  define BENCH_HAS_OPENCV_TRACKING 1
#else
#  define BENCH_HAS_OPENCV_TRACKING 0
#endif

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

// 파이프라인이 매 프레임 내놓는 타깃 출처를 4개 상태로 환원한다.
enum class FrameState { None, ByteTrack, Csrt, Predict };

const char* stateName(FrameState s) {
    switch (s) {
        case FrameState::None:      return "none";
        case FrameState::ByteTrack: return "bytetrack";
        case FrameState::Csrt:      return "csrt";
        case FrameState::Predict:   return "kalman_hold";
    }
    return "unknown";
}

FrameState toState(const ptcamera::PipelineResult& result) {
    if (!result.target) {
        return FrameState::None;
    }
    switch (result.target->source) {
        case ptcamera::TrackSource::ByteTrack:
        case ptcamera::TrackSource::Detector:
            return FrameState::ByteTrack;
        case ptcamera::TrackSource::OpenCV:
            return FrameState::Csrt;
        case ptcamera::TrackSource::Predict:
            return FrameState::Predict;
    }
    return FrameState::None;
}

std::string readStringArg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[i]);
    }
    return argv[++i];
}

int readIntArg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[i]);
    }
    return std::atoi(argv[++i]);
}

float readFloatArg(int argc, char** argv, int& i) {
    if (i + 1 >= argc) {
        throw std::runtime_error(std::string("missing value for ") + argv[i]);
    }
    return std::stof(argv[++i]);
}

void printUsage() {
    std::cout <<
        "Usage: benchmark --video <path> [--camera N] [--model path] [--device CUDA|CPU]\n"
        "                 [--conf 0.25] [--csv out.csv] [--dt 0.033] [--max-frames N]\n"
        "                 [--display]\n"
        "  --video       입력 영상 파일(재현 가능한 벤치마크 권장)\n"
        "  --camera      영상 대신 카메라 인덱스로 측정(비재현)\n"
        "  --csv         프레임별 로그 CSV 출력 경로\n"
        "  --dt          프레임 간 dt 강제값(미지정 시 영상 FPS 사용)\n"
        "  --max-frames  처리할 최대 프레임 수\n"
        "  --display     처리 화면 표시(기본 off, 헤드리스)\n";
}

double pct(long long n, long long total) {
    return total > 0 ? 100.0 * static_cast<double>(n) / static_cast<double>(total) : 0.0;
}

}  // namespace

int main(int argc, char** argv) {
    ptcamera::TrackerSettings settings = ptcamera::defaultSettings();

    std::string videoPath;
    int cameraIndex = -1;
    std::string csvPath;
    double forcedDt = 0.0;
    long long maxFrames = -1;
    bool display = false;

    try {
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--video") {
                videoPath = readStringArg(argc, argv, i);
            } else if (arg == "--camera") {
                cameraIndex = readIntArg(argc, argv, i);
            } else if (arg == "--model") {
                settings.modelPath = readStringArg(argc, argv, i);
            } else if (arg == "--device") {
                settings.inferenceDevice = readStringArg(argc, argv, i);
            } else if (arg == "--conf") {
                settings.confidenceThreshold = readFloatArg(argc, argv, i);
            } else if (arg == "--csv") {
                csvPath = readStringArg(argc, argv, i);
            } else if (arg == "--dt") {
                forcedDt = std::stod(readStringArg(argc, argv, i));
            } else if (arg == "--max-frames") {
                maxFrames = std::atoll(readStringArg(argc, argv, i).c_str());
            } else if (arg == "--display") {
                display = true;
            } else if (arg == "--help") {
                printUsage();
                return 0;
            } else {
                throw std::runtime_error("unknown argument: " + arg);
            }
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        printUsage();
        return 2;
    }

    if (videoPath.empty() && cameraIndex < 0) {
        std::cerr << "입력이 없습니다. --video <path> 또는 --camera <N> 지정.\n";
        printUsage();
        return 2;
    }

#if !BENCH_HAS_OPENCV_TRACKING
    std::cout << "[주의] 이 빌드에는 OpenCV contrib 트래커(CSRT)가 없습니다. "
                 "CSRT 폴백은 비활성이며 csrt 점유율은 항상 0으로 집계됩니다.\n";
#endif

    try {
        cv::VideoCapture cap;
        double videoFps = 0.0;
        if (!videoPath.empty()) {
            cap.open(videoPath);
            if (!cap.isOpened()) {
                std::cerr << "영상을 열 수 없습니다: " << videoPath << '\n';
                return 1;
            }
            videoFps = cap.get(cv::CAP_PROP_FPS);
        } else {
            cap.open(cameraIndex);
            if (!cap.isOpened()) {
                std::cerr << "카메라를 열 수 없습니다: " << cameraIndex << '\n';
                return 1;
            }
            if (settings.cameraWidth > 0)  cap.set(cv::CAP_PROP_FRAME_WIDTH, settings.cameraWidth);
            if (settings.cameraHeight > 0) cap.set(cv::CAP_PROP_FRAME_HEIGHT, settings.cameraHeight);
            videoFps = 30.0;
        }
        if (videoFps <= 1.0 || videoFps > 1000.0) {
            videoFps = 30.0;
        }
        const double dt = forcedDt > 0.0 ? forcedDt : 1.0 / videoFps;

        std::cout << "벤치마크 시작\n";
        std::cout << "입력: " << (videoPath.empty() ? ("camera " + std::to_string(cameraIndex)) : videoPath) << '\n';
        std::cout << "모델: " << settings.modelPath << " (" << settings.inferenceDevice << ")\n";
        std::cout << "conf=" << settings.confidenceThreshold
                  << "  dt=" << dt << "s (" << (1.0 / dt) << " Hz)\n";

        ptcamera::TrackingPipeline pipeline(settings);

        std::ofstream csv;
        if (!csvPath.empty()) {
            csv.open(csvPath);
            csv << "frame,t,state,track_id,cx,cy,box_x,box_y,box_w,box_h,conf,num_det,num_tracks\n";
        }

        // 집계 카운터
        long long total = 0;
        long long stateCount[4] = {0, 0, 0, 0};  // None, ByteTrack, Csrt, Predict
        long long idSwitches = 0;
        long long reacquisitions = 0;     // 손실 후 다시 타깃 확보
        long long holdEntries = 0;        // 비홀드 -> Kalman 홀드 전이 횟수
        long long holdRecovered = 0;      // 홀드 구간이 타깃 복귀로 끝난 횟수
        long long holdLost = 0;           // 홀드 구간이 완전 손실로 끝난 횟수
        long long holdFrameSum = 0;       // 누적 홀드 프레임(평균 계산용)
        int holdRunLen = 0;
        int maxHoldRun = 0;
        double confSum = 0.0;             // ByteTrack 프레임 신뢰도 평균용
        long long confN = 0;

        // 추적 구간(fragment) 길이
        long long fragments = 0;
        int fragRunLen = 0;
        int maxFragRun = 0;
        long long fragFrameSum = 0;

        FrameState prevState = FrameState::None;
        bool prevHadTarget = false;
        int prevTrackId = -1;

        cv::Mat frame;
        while (cap.read(frame) && !frame.empty()) {
            if (maxFrames >= 0 && total >= maxFrames) {
                break;
            }
            const long long idx = total;
            const double t = static_cast<double>(idx) * dt;

            ptcamera::PipelineResult result = pipeline.update(frame, dt);
            const FrameState state = toState(result);
            const bool hasTarget = (state != FrameState::None);
            const int trackId = hasTarget ? result.target->trackId : -1;

            ++total;
            ++stateCount[static_cast<int>(state)];

            // 신뢰도(실측 탐지 = ByteTrack 프레임만 의미 있음)
            if (state == FrameState::ByteTrack) {
                confSum += result.target->confidence;
                ++confN;
            }

            // ID 전환: 연속으로 타깃이 잡힌 상태에서 trackId가 바뀐 경우만.
            if (hasTarget && prevHadTarget && prevTrackId >= 0 && trackId >= 0 &&
                trackId != prevTrackId) {
                ++idSwitches;
            }
            // 재확보: 손실 직후 다시 타깃을 잡음.
            if (hasTarget && !prevHadTarget && total > 1) {
                ++reacquisitions;
            }

            // Kalman 홀드 구간 추적
            if (state == FrameState::Predict) {
                if (prevState != FrameState::Predict) {
                    ++holdEntries;
                    holdRunLen = 0;
                }
                ++holdRunLen;
                ++holdFrameSum;
                maxHoldRun = std::max(maxHoldRun, holdRunLen);
            } else if (prevState == FrameState::Predict) {
                // 홀드 종료: 무엇으로 끝났는지 기록
                if (hasTarget) {
                    ++holdRecovered;
                } else {
                    ++holdLost;
                }
                holdRunLen = 0;
            }

            // 추적 구간(fragment): 타깃이 끊김 없이 유지되는 구간
            if (hasTarget) {
                if (!prevHadTarget) {
                    ++fragments;
                    fragRunLen = 0;
                }
                ++fragRunLen;
                ++fragFrameSum;
                maxFragRun = std::max(maxFragRun, fragRunLen);
            }

            if (csv.is_open()) {
                if (hasTarget) {
                    const auto& b = result.target->box;
                    char line[256];
                    std::snprintf(line, sizeof(line),
                        "%lld,%.4f,%s,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%zu,%zu\n",
                        idx, t, stateName(state), trackId,
                        result.target->center.x, result.target->center.y,
                        b.x, b.y, b.width, b.height, result.target->confidence,
                        result.detections.size(), result.tracks.size());
                    csv << line;
                } else {
                    char line[128];
                    std::snprintf(line, sizeof(line),
                        "%lld,%.4f,%s,,,,,,,,,%zu,%zu\n",
                        idx, t, stateName(state),
                        result.detections.size(), result.tracks.size());
                    csv << line;
                }
            }

            if (display) {
                cv::imshow("benchmark", frame);
                if (cv::waitKey(1) == 27) {
                    break;
                }
            }

            if (total % 200 == 0) {
                std::cout << "  처리 " << total << " 프레임...\r" << std::flush;
            }

            prevState = state;
            prevHadTarget = hasTarget;
            prevTrackId = trackId;
        }

        // 마지막 프레임이 홀드로 끝났으면 종결 처리
        if (prevState == FrameState::Predict) {
            ++holdLost;
        }

        const long long targetFrames = stateCount[1] + stateCount[2] + stateCount[3];
        const double avgHold = holdEntries > 0
                                   ? static_cast<double>(holdFrameSum) / holdEntries : 0.0;
        const double avgFrag = fragments > 0
                                   ? static_cast<double>(fragFrameSum) / fragments : 0.0;
        const double avgConf = confN > 0 ? confSum / confN : 0.0;

        std::cout << "\n\n================ 추적 파이프라인 정량화 결과 ================\n";
        std::cout << "전체 프레임            : " << total << "\n";
        std::cout << "타깃 존재(커버리지)    : " << targetFrames
                  << "  (" << pct(targetFrames, total) << "%)\n";
        std::cout << "\n[ 모드 점유율 (타깃 출처별) ]\n";
        std::cout << "  ByteTrack(실측 추적) : " << stateCount[1]
                  << "  (" << pct(stateCount[1], total) << "%)\n";
        std::cout << "  CSRT(시각 폴백)      : " << stateCount[2]
                  << "  (" << pct(stateCount[2], total) << "%)\n";
        std::cout << "  Kalman 홀드(예측)    : " << stateCount[3]
                  << "  (" << pct(stateCount[3], total) << "%)\n";
        std::cout << "  손실(타깃 없음)      : " << stateCount[0]
                  << "  (" << pct(stateCount[0], total) << "%)\n";
        std::cout << "\n[ Kalman 홀드 전환 ]\n";
        std::cout << "  홀드 진입 횟수       : " << holdEntries << "\n";
        std::cout << "    - 타깃 복귀로 종료 : " << holdRecovered << "\n";
        std::cout << "    - 완전 손실로 종료 : " << holdLost << "\n";
        std::cout << "  평균 홀드 길이       : " << avgHold << " 프레임\n";
        std::cout << "  최장 홀드 길이       : " << maxHoldRun << " 프레임\n";
        std::cout << "\n[ ID / 연속성 ]\n";
        std::cout << "  ID 전환 횟수         : " << idSwitches << "\n";
        std::cout << "  재확보 횟수          : " << reacquisitions << "\n";
        std::cout << "  추적 구간 수         : " << fragments << "\n";
        std::cout << "  평균 추적 구간 길이  : " << avgFrag << " 프레임\n";
        std::cout << "  최장 연속 추적       : " << maxFragRun << " 프레임\n";
        std::cout << "\n[ 신뢰도 ]\n";
        std::cout << "  ByteTrack 평균 conf  : " << avgConf << "\n";
        std::cout << "============================================================\n";

        if (csv.is_open()) {
            std::cout << "프레임별 로그: " << csvPath << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }

    return 0;
}
