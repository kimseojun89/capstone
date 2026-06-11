# 추적 파이프라인 정량화 (tracking_benchmark)

`antidrone/cpp`의 TrackingPipeline(YOLOv8x → ByteTrack → CSRT → Kalman 홀드)이
**실제로 어떤 경로로 타깃을 추적하는지**를 정답 라벨(GT) 없이 정량화한 결과 모음.

## 폴더 구성

| 파일 | 내용 |
|---|---|
| `bench_result.csv` | 프레임별 로그 (benchmark.exe 출력) |
| `summary.txt` | 집계 요약 리포트 |
| `dashboard.png` | 모드 타임라인 + 점유율 + trackID + conf + bbox + 요약 |
| `trajectory.png` | 타깃 센터 궤적(화면 좌표, 상태별 색) |
| `visualize.py` | CSV → PNG 시각화 스크립트 (pandas 불필요) |

## 측정 원리

파이프라인이 매 프레임 반환하는 `result.target->source`(TrackSource)가
타깃이 어느 단계에서 나왔는지 라벨한다. 이를 4개 상태로 환원:
`ByteTrack(실측) / CSRT(시각 폴백) / Kalman 홀드(예측) / 손실`.
여기서 모드 점유율·홀드 전환 빈도·ID 전환·커버리지·연속성을 모두 집계한다.

> ⚠️ **self-report 지표**다 — 파이프라인이 "스스로 어느 경로를 썼나"는 정확히 잰다.
> 진짜 위치 정확도(IoU 성공률·MOTA·IDF1)는 라벨된 GT 영상이 별도로 필요하다.
>
> ⚠️ 현재 PC 빌드는 OpenCV contrib가 없어 **CSRT가 컴파일 제외**됐다.
> 따라서 CSRT 점유율은 항상 0이며, ByteTrack 실패 시 바로 Kalman 홀드로 간다.

## 재생성 방법

```powershell
# 1) 빌드 (benchmark.exe 포함)
cd C:\Users\kimse\capstone
.\scripts\build_win.bat

# 2) 영상 파일 디코딩용 OpenCV ffmpeg DLL을 PATH에 추가
$env:PATH = "C:\opencv\build\x64\vc16\bin;" + $env:PATH

# 3) 벤치마크 실행 → CSV 생성
$video = "C:\Users\kimse\capstone\scripts\Drone-Detection-YOLOv8x-main\test\pexels-joseph-redfield-8459631 (1080p).mp4"
.\antidrone\cpp\build_win\benchmark.exe --video $video `
    --csv .\scripts\tracking_benchmark\bench_result.csv

# 4) 시각화 → dashboard.png, trajectory.png
python .\scripts\tracking_benchmark\visualize.py .\scripts\tracking_benchmark\bench_result.csv
```

## 이번 결과 요약 (테스트 영상, 552프레임)

- 커버리지 100%, 전부 ByteTrack 경로, 평균 conf 0.77 → 탐지기는 드론을 한 번도 놓치지 않음.
- Kalman 홀드/손실 0회 → 폴백 경로가 발동할 일이 없었음.
- **ID 전환 66회** → 탐지는 안정적이나 ByteTrack의 track ID 일관성이 약함.
  ID 안정성이 중요하면 `bytetrack_drone.yaml`의 `match_thresh`/`track_buffer` 튜닝 검토.
