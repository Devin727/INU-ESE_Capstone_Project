# 초음파 균열 감지 원리와 A-scan 분석 조사

원본 Markdown · **[docs/ultrasonic_crack_detection_principle.md](../docs/ultrasonic_crack_detection_principle.md)**

5 MHz 단일 소자 수직 탐촉자의 반사파를 정상 기준 파형과 비교해 균열 후보를 찾는 방법을 조사했다. 아래 C++ 코드는 참고 구현이며, 원문에는 컴파일·시뮬레이션·실측 검증 결과가 없다. 게이트와 임계값은 실험으로 조정할 예시값이다.

## 검사 원리와 표현 방식

탐촉자로 배관에 초음파를 송신한 뒤 반사파를 수신한다. 수신 파형의 진폭, 도착 시간과 정상 기준 대비 변화를 분석해 균열 의심 여부를 판단하는 흐름이다.

| 방식 | 조사 내용 |
| --- | --- |
| A-scan | 시간에 따른 반사파 진폭을 나타낸다. 아래 예제의 분석 대상이다. |
| C-scan | 검사 지점별 결과를 평면 영상으로 나타내는 방식으로 조사했다. |
| 배관 위치 연결 | 인코더·IMU·탐촉자 각도·ToF 정보를 검사 결과와 함께 기록하는 구상이다. |

원문은 A-scan과 ToF의 병용을 제안하지만, 예제는 위치 추정이나 C-scan 생성까지 구현하지 않는다.

## 시스템 구성

```text
함수 발생기/펄서
    → 송수신 보호(T/R) 회로 + 초음파 탐촉자
    → 수신부 + 오실로스코프 또는 ADC
    → A-scan 신호 처리
    → 결함 후보와 측정 위치 연결
```

파형 수집 장치의 샘플링 속도를 확인하고, 정상·검사 파형을 비교할 수 있는 조건으로 수집해야 한다. 실제 수집 장치와 통신·동기화 방식은 이 조사에서 구현하지 않았다.

## 신호 처리 순서

| 단계 | 처리 |
| --- | --- |
| 파형 수집 | 정상 기준과 검사 지점의 A-scan을 수집한다. |
| 대역통과 필터 | 5 MHz 중심 주파수 주변을 남겨 대역 밖 잡음을 제거한다. |
| Hilbert 변환 | 해석 신호의 크기로 포락선(envelope)을 구한다. |
| 특징 추출 | 배면 피크, 도착 시간, 추가 에코, 에너지와 파형 차이를 검토한다. |
| 정상 기준 비교 | 기준 대비 진폭 감소·결함 구간 증가·파형 변화를 계산한다. |
| 임계값 판정 | 예시 기준을 넘는 항목 수로 균열 의심 여부를 표시한다. |

원문에서 조사한 다섯 특징과 코드의 구현 범위는 구분할 필요가 있다.

| 특징 | 예제 코드의 처리 |
| --- | --- |
| Backwall Peak | 지정 게이트에서 배면 반사 피크를 추출하고 비교한다. |
| Echo Arrival Time | 배면 피크의 시각을 저장하지만 최종 점수에는 사용하지 않는다. |
| Extra Echo Peak | 별도의 추가 에코 피크 검출 함수는 없다. |
| Signal Energy | 결함 게이트의 포락선 제곱 평균을 사용한다. 시간 적분 에너지와 구분한다. |
| Waveform Difference | 전체 포락선의 기준 대비 정규화 차이를 계산한다. |

## 비교 지표와 예시 판정 기준

```text
amplitudeDrop = 1 − 검사 배면 피크 / 정상 배면 피크

energyIncrease = 검사 결함 구간 평균 제곱 포락선
                 / max(정상 결함 구간 평균 제곱 포락선, 1e−12) − 1

waveformDifference = sqrt(Σ(검사 포락선 − 정상 포락선)²
                         / max(Σ정상 포락선², 1e−12))
```

| 파라미터 | 예제 값 | 상태 |
| --- | --- | --- |
| 필터 대역 | 3~7 MHz | 초기 예시 |
| 배면 반사 게이트 | 2.0~4.0 µs | 실측 후 수정 |
| 결함 게이트 | 0.5~2.0 µs | 실측 후 수정 |
| 배면 피크 감소 | 0.40 초과 | 미보정 임계값 |
| 결함 구간 증가 | 0.50 초과 | 미보정 임계값 |
| 파형 차이 | 0.25 초과 | 미보정 임계값 |
| 최종 판정 | 3개 중 2개 이상 초과 | 균열 의심 후보 |

두 시간 게이트는 2.0 µs 경계를 함께 포함한다. 실제 반사 위치와 게이트 경계를 확인한 뒤 사용한다. 접촉 상태 변화와 실제 결함을 구분하는 성능이나 검출률은 검증되지 않았다.

## 참고 C++ 구현

원문의 전체 코드를 보존했다. FFTW3로 필터와 포락선을 계산하고 `normal.csv`, `test.csv`를 비교한다. 파일 입력, 특징 추출과 판정의 역할은 다음과 같다.

| 함수 | 역할 |
| --- | --- |
| `loadCSV()` | 시간·전압 두 열을 읽는다. |
| `bandPassFilter()` | 평균 제거 후 FFT 기반 대역통과 필터를 적용한다. |
| `hilbertEnvelope()` | Hilbert 변환 기반 포락선을 구한다. |
| `extractFeatures()` | 시간 게이트별 배면 피크·시각·제곱 평균을 구한다. |
| `compareSignals()` | 정상 기준과의 세 비교 지표를 계산한다. |
| `detectCrack()` | 임계값 초과 항목 수로 의심 여부를 정한다. |
| `main()` | 시간축 검사, 분석, 화면 출력과 결과 CSV 저장을 연결한다. |

```cpp
#include <algorithm>
#include <cmath>
#include <complex>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>
#include <fftw3.h>

using Signal = std::vector<double>;

constexpr double PI = 3.14159265358979323846;

// CSV 형식: time_s,voltage_v
struct AScan {
    Signal time;
    Signal voltage;
};

struct Features {
    double backwallPeak;
    double backwallTimeUs;
    double defectEnergy;
};

struct Comparison {
    double amplitudeDrop;
    double energyIncrease;
    double waveformDifference;
};

struct Detection {
    bool crackSuspected;
    int score;
};

// ========================================
// CSV 데이터 읽기
// ========================================

AScan loadCSV(const std::string& filename) {
    std::ifstream file(filename);

    if (!file) {
        throw std::runtime_error(
            "Cannot open: " + filename
        );
    }

    AScan data;
    std::string line;

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string a, b;

        if (!std::getline(ss, a, ',')) continue;
        if (!std::getline(ss, b, ',')) continue;

        try {
            double t = std::stod(a);
            double v = std::stod(b);

            data.time.push_back(t);
            data.voltage.push_back(v);
        }
        catch (...) {
            // CSV header 건너뛰기
        }
    }

    if (data.time.size() < 32) {
        throw std::runtime_error(
            "Not enough samples"
        );
    }

    return data;
}

// ========================================
// 4.1 Band-pass Filter
// FFT 기반 대역통과 필터
// ========================================

Signal bandPassFilter(
    const Signal& input,
    double fs,
    double lowCut,
    double highCut
) {
    const int N = static_cast<int>(input.size());

    if (N == 0 || fs <= 0 ||
        lowCut <= 0 || highCut <= lowCut ||
        highCut >= fs / 2.0) {
        throw std::invalid_argument(
            "Invalid filter parameters"
        );
    }

    fftw_complex* in = fftw_alloc_complex(N);
    fftw_complex* out = fftw_alloc_complex(N);

    if (!in || !out) {
        throw std::runtime_error("FFTW allocation failed");
    }

    double mean = std::accumulate(
        input.begin(), input.end(), 0.0
    ) / N;

    for (int i = 0; i < N; ++i) {
        in[i][0] = input[i] - mean;
        in[i][1] = 0.0;
    }

    fftw_plan forward = fftw_plan_dft_1d(
        N, in, out, FFTW_FORWARD, FFTW_ESTIMATE
    );

    fftw_plan inverse = fftw_plan_dft_1d(
        N, out, in, FFTW_BACKWARD, FFTW_ESTIMATE
    );

    if (!forward || !inverse) {
        throw std::runtime_error("FFTW plan failed");
    }

    fftw_execute(forward);

    // 주파수 영역에서 대역 밖 제거
    for (int k = 0; k < N; ++k) {
        double frequency =
            (k <= N / 2 ? k : k - N) * fs / N;

        if (std::abs(frequency) < lowCut ||
            std::abs(frequency) > highCut) {
            out[k][0] = 0.0;
            out[k][1] = 0.0;
        }
    }

    fftw_execute(inverse);

    Signal filtered(N);

    for (int i = 0; i < N; ++i) {
        filtered[i] = in[i][0] / N;
    }

    fftw_destroy_plan(forward);
    fftw_destroy_plan(inverse);
    fftw_free(in);
    fftw_free(out);

    return filtered;
}

// ========================================
// 4.2 Hilbert Transform
// Analytic Signal -> Envelope
// ========================================

Signal hilbertEnvelope(const Signal& input) {
    const int N = static_cast<int>(input.size());

    if (N == 0) return {};

    fftw_complex* in = fftw_alloc_complex(N);
    fftw_complex* out = fftw_alloc_complex(N);

    if (!in || !out) {
        throw std::runtime_error("FFTW allocation failed");
    }

    for (int i = 0; i < N; ++i) {
        in[i][0] = input[i];
        in[i][1] = 0.0;
    }

    fftw_plan forward = fftw_plan_dft_1d(
        N, in, out, FFTW_FORWARD, FFTW_ESTIMATE
    );

    fftw_plan inverse = fftw_plan_dft_1d(
        N, out, in, FFTW_BACKWARD, FFTW_ESTIMATE
    );

    if (!forward || !inverse) {
        throw std::runtime_error("FFTW plan failed");
    }

    fftw_execute(forward);

    for (int k = 0; k < N; ++k) {
        double scale = 0.0;

        if (k == 0) {
            scale = 1.0;
        }
        else if (N % 2 == 0 && k == N / 2) {
            scale = 1.0;
        }
        else if (k <= (N - 1) / 2) {
            scale = 2.0;
        }

        out[k][0] *= scale;
        out[k][1] *= scale;
    }

    fftw_execute(inverse);

    Signal envelope(N);

    for (int i = 0; i < N; ++i) {
        double re = in[i][0] / N;
        double im = in[i][1] / N;

        envelope[i] = std::hypot(re, im);
    }

    fftw_destroy_plan(forward);
    fftw_destroy_plan(inverse);
    fftw_free(in);
    fftw_free(out);

    return envelope;
}

// ========================================
// Envelope 특징 추출
// ========================================

Features extractFeatures(
    const Signal& envelope,
    const Signal& time,
    double backStartUs,
    double backEndUs,
    double defectStartUs,
    double defectEndUs
) {
    if (envelope.size() != time.size()) {
        throw std::invalid_argument(
            "Envelope/time size mismatch"
        );
    }

    double peak = -1.0;
    double peakTime = 0.0;
    double energy = 0.0;

    int backCount = 0;
    int defectCount = 0;

    for (size_t i = 0; i < envelope.size(); ++i) {
        double tUs = time[i] * 1e6;

        if (tUs >= backStartUs && tUs <= backEndUs) {
            ++backCount;

            if (envelope[i] > peak) {
                peak = envelope[i];
                peakTime = tUs;
            }
        }

        if (tUs >= defectStartUs &&
            tUs <= defectEndUs) {
            energy += envelope[i] * envelope[i];
            ++defectCount;
        }
    }

    if (backCount == 0 || defectCount == 0) {
        throw std::runtime_error(
            "Echo gate contains no samples"
        );
    }

    // Gate 내 평균 제곱 Envelope
    energy /= defectCount;

    return {peak, peakTime, energy};
}

// ========================================
// 4.3 Baseline Comparison
// ========================================

Comparison compareSignals(
    const Signal& normalEnvelope,
    const Signal& testEnvelope,
    const Features& normal,
    const Features& test
) {
    if (normalEnvelope.size() != testEnvelope.size() ||
        normal.backwallPeak <= 1e-12) {
        throw std::invalid_argument(
            "Invalid baseline"
        );
    }

    double amplitudeDrop =
        1.0 - test.backwallPeak / normal.backwallPeak;

    double energyIncrease =
        test.defectEnergy /
        std::max(normal.defectEnergy, 1e-12) - 1.0;

    double diffSum = 0.0;
    double refSum = 0.0;

    for (size_t i = 0; i < normalEnvelope.size(); ++i) {
        double d =
            testEnvelope[i] - normalEnvelope[i];

        diffSum += d * d;
        refSum += normalEnvelope[i] * normalEnvelope[i];
    }

    double waveformDifference =
        std::sqrt(diffSum /
                  std::max(refSum, 1e-12));

    return {
        amplitudeDrop,
        energyIncrease,
        waveformDifference
    };
}

// ========================================
// 4.4 Threshold Detection
// 임계값은 실험 데이터로 조정해야 함
// ========================================

Detection detectCrack(const Comparison& c) {
    constexpr double AMP_DROP = 0.40;
    constexpr double ENERGY_INCREASE = 0.50;
    constexpr double WAVEFORM_DIFF = 0.25;

    int score = 0;

    if (c.amplitudeDrop > AMP_DROP)
        ++score;

    if (c.energyIncrease > ENERGY_INCREASE)
        ++score;

    if (c.waveformDifference > WAVEFORM_DIFF)
        ++score;

    return {score >= 2, score};
}

// ========================================
// 메인 프로그램
// ========================================

int main() {
    try {
        AScan normal = loadCSV("normal.csv");
        AScan test = loadCSV("test.csv");

        if (normal.time.size() != test.time.size()) {
            throw std::runtime_error(
                "Sample count mismatch"
            );
        }

        const int N =
            static_cast<int>(normal.time.size());

        // 시간축 및 샘플링 간격 검사
        double dt = normal.time[1] - normal.time[0];

        if (dt <= 0) {
            throw std::runtime_error(
                "Invalid sampling interval"
            );
        }

        for (int i = 1; i < N; ++i) {
            double dtn =
                normal.time[i] - normal.time[i - 1];

            double dtt =
                test.time[i] - test.time[i - 1];

            if (std::abs(dtn - dt) > dt * 0.001 ||
                std::abs(dtt - dt) > dt * 0.001 ||
                std::abs(test.time[i] - normal.time[i])
                    > dt * 0.001) {
                throw std::runtime_error(
                    "Time axes must be aligned"
                );
            }
        }

        if (std::abs(test.time[0] - normal.time[0])
            > dt * 0.001) {
            throw std::runtime_error(
                "Time axes must be aligned"
            );
        }

        double fs = 1.0 / dt;

        // 4.1 Band-pass Filter
        // 초기 예시: 3~7MHz
        Signal normalFiltered = bandPassFilter(
            normal.voltage, fs, 3e6, 7e6
        );

        Signal testFiltered = bandPassFilter(
            test.voltage, fs, 3e6, 7e6
        );

        // 4.2 Hilbert Envelope
        Signal normalEnvelope =
            hilbertEnvelope(normalFiltered);

        Signal testEnvelope =
            hilbertEnvelope(testFiltered);

        // 예시 Gate (실측 후 수정)
        Features normalFeatures = extractFeatures(
            normalEnvelope, normal.time,
            2.0, 4.0, 0.5, 2.0
        );

        Features testFeatures = extractFeatures(
            testEnvelope, test.time,
            2.0, 4.0, 0.5, 2.0
        );

        // 4.3 Baseline Comparison
        Comparison comparison = compareSignals(
            normalEnvelope,
            testEnvelope,
            normalFeatures,
            testFeatures
        );

        // 4.4 Threshold Detection
        Detection result = detectCrack(comparison);

        std::cout << std::fixed
                  << std::setprecision(4);

        std::cout << "=== A-SCAN ANALYSIS ===\n";
        std::cout << "Sampling rate: "
                  << fs / 1e6 << " MS/s\n";

        std::cout << "Normal backwall peak: "
                  << normalFeatures.backwallPeak << "\n";

        std::cout << "Test backwall peak: "
                  << testFeatures.backwallPeak << "\n";

        std::cout << "Amplitude drop: "
                  << comparison.amplitudeDrop << "\n";

        std::cout << "Defect energy increase: "
                  << comparison.energyIncrease << "\n";

        std::cout << "Waveform difference: "
                  << comparison.waveformDifference << "\n";

        std::cout << "Detection score: "
                  << result.score << "/3\n";

        std::cout << "Result: "
                  << (result.crackSuspected
                      ? "CRACK SUSPECTED"
                      : "NO CRACK INDICATION")
                  << "\n";

        // 분석된 Envelope를 CSV로 저장
        std::ofstream output("envelope_result.csv");
        output << "time_s,normal_env,test_env\n";

        for (int i = 0; i < N; ++i) {
            output << normal.time[i] << ","
                   << normalEnvelope[i] << ","
                   << testEnvelope[i] << "\n";
        }

        std::cout << "Saved: envelope_result.csv\n";
    }
    catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}

```

## 실행 방법과 입력 조건

원문이 제시한 Linux 또는 WSL 환경 기준이다. 위 코드를 `main.cpp`로 저장한 뒤 실행한다.

```bash
sudo apt update
sudo apt install g++ libfftw3-dev
g++ -std=c++17 -O2 main.cpp -lfftw3 -lm -o ascan
./ascan
```

프로그램을 실행하는 폴더에 `normal.csv`, `test.csv`를 둔다. 각 행은 `time_s,voltage_v` 형식이다. 원문에는 시험용 CSV가 포함되어 있지 않다.

- 최소 32개 샘플과 두 파형의 동일한 샘플 수를 요구한다.
- 양의 샘플 간격과 정렬된 시간축을 검사한다. 예제의 허용 차이는 기준 간격의 약 0.1%다.
- 필터 함수는 상한 주파수가 샘플링률의 절반보다 작아야 한다고 검사한다. 3~7 MHz 예제는 코드 조건상 14 MS/s보다 큰 샘플링률이 필요하다.
- 성공하면 비교 수치·점수·의심 여부를 출력하고 `envelope_result.csv`를 저장하도록 작성되어 있다.

CSV 변환에 실패한 행을 헤더처럼 건너뛰는 동작, 비유한 입력값 처리, FFTW 실패 경로의 자원 정리와 결과 파일 저장 여부는 실제 사용 전에 검토할 항목이다. 이번 문서 정리에서 예제를 컴파일하거나 파형 분석을 실행하지 않았다.

## 검사 결과와 배관 위치 연결

원문은 다음 데이터 구조를 제안한다.

```cpp
struct CrackRecord {
    double distanceM;     // Encoder 기반 이동 거리
    double headingDeg;    // IMU 기반 진행 방향
    double probeAngleDeg; // 탐촉자 검사 방향
    double tofDistanceMm; // ToF 거리
    int detectionScore;
    bool crackSuspected;
};
```

구조체는 기록 형식 제안이며 위 `main()`에서 사용하지 않는다. ToF 거리와 A-scan 반사파 도착 시간은 서로 다른 항목이다.
실제 통합 시에는 측정 시점의 위치·자세·탐촉자 상태와 파형을 연결하고, 배관 구간 ID·측정 ID·수집 시각·원파형 경로를 정해야 한다.

## 후속 검증과 관련 문서

- 정상·결함 시편에서 같은 조건으로 원파형을 반복 수집한다.
- 실제 샘플링 조건과 반사 위치에 맞게 대역·게이트를 정한다.
- 실측 분포로 임계값을 보정하고 접촉 불량·오경보를 평가한다.
- 조사한 추가 에코 피크와 도착 시간을 실제 판정에 반영할지 결정한다.
- 위치 추정과 파형 수집 시점을 동기화해 검사 결과를 지도에 연결한다.

관련 기록 · [초음파 벤치 실험 준비](2026-10-13-ultrasonic-experiment-preparation.md), [만능기판 펄서·리시버 설계](2026-10-23-perfboard-pulser-receiver.md), [배관 위치 추정과 센서 융합 조사](related_mapping.md)
