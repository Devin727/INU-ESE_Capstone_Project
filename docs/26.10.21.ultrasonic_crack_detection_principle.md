# 초음파 탐촉자 균열 감지 알고리즘 조사


#### 1. 초음파 탐촉자 균열 감지 원리
(*5MHz Single Element 수직 탐촉자 기준)
탐촉자가 초음파를 배관에 쏘고, 다시 초음파를 수신
수신된 신호를 비교하여 크랙 유무를 판단한다.

#### 2. 초음파 검사 방식
2.1.  A-scan(진폭-시간 그래프) ← 매핑은 안되기에, ToF같이 사용해서 매핑
2.2. C-scan(평면 영상) ← 2차원 매핑은  c스캔 방식


#### 3. 시스템 구성
function generator/pulser -> T/R 회로 + 초음파 탐촉자 -> 리서버+오실로스코프 /ADC -> A-scan 신호 처리 -> 결함 위치 매핑


#### 4. 사용 알고리즘

4.1. Band-pass Filter
초음파 측정 신호에는 전기적 잡음, 기계적 진동, 다른 주파수 성분이 포함될 수 있기에,
5MHz 주변 신호 통과시켜 대역 밖 잡음 제거

4.2. Hilbert Transform
A-scan의 진폭 Envelope 추출
→ Envelope에서 특징 5가지 추출(이미지 참고)

4.3. Baseline Comparison
정상 배관 기준 신호와 비교

4.4. Threshold Detection
특징값을 기준으로 균열 의심 여부 판단

```jsx
A-scan 데이터 수집
: 함수 발생기로 탐촉자 구동 후 오실로스코프로 반사파 측정
*샘플링 속도 주의

↓ 

Band-pass Filter 적용
: 탐촉자의 중심 주파수인 5MHz 부근만 통과

↓ 

Hilbert Transform 적용
: 필터링한 신호에 적용하여 포락선(Envelope) 추출 (초음파 신호의 진폭을 알기위해)

↓ 

Echo 특징 추출
: Envelope에서 Backwall Peak, Echo Arrival Time, 
Extra Echo Peak, Signal Energy, Waveform Difference 추출

↓

정상 기준 파형과 비교
: 두 신호가 유사하면 정상일 가능성이 높고, 차이가 크면 이상

↓

균열 판정

```

#### 5. 코드 구현 

#### 전체 코드 (시뮬 필요)

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


#### 코드 실행 방법
1. Linux 또는 WSL 환경에서 FFTW3 개발 라이브러리를 설치

```bash
sudo apt update
sudo apt install g++ libfftw3-dev
```

1. 코드를  main.cpp로 저장한 뒤 컴파일하기
```bash
g++ -std=c++17 -O2 main.cpp -lfftw3 -lm -o ascan
./ascan
```

프로그램과 같은 폴더에 `normal.csv`, `test.csv` 파일을 넣어야 함.


#### 배관 매핑과 연결하는 코드

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
