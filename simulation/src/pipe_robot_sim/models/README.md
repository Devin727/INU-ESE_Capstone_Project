# models

차체 모델 폴더를 여기에 둡니다 (`models/<이름>/<이름>.urdf.xacro` + `robot.yaml`).

- `snake_v`: 뱀형 V자 (PIRATE 방식) 기준 모델. 수평 좌·우, 위 90° 엘보 통과 (oracle 제어)
- `snake_v2`: `snake_v` + 실제 부품 + 꼬리 끝 탐촉자 (D안). 엘보 통과 미해결
- `common/common.xacro`: 공통 매크로 (관성, 옴니휠 롤러, 시뮬 플러그인)

새 모델이 지켜야 할 토픽, 플러그인, `robot.yaml` 키는 [docs/02 차체 모델](../../../docs/02_차체_모델.md)을 보세요.
