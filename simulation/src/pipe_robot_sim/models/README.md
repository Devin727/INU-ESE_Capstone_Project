# models

차체 모델 폴더를 여기에 둡니다 (`models/<이름>/<이름>.urdf.xacro` + `robot.yaml`).

- `snake_v`: 뱀형 V자 (PIRATE 방식) 기준 모델. 수평 좌·우, 위 90° 엘보 통과 (oracle 제어)
- `snake_v2`: `snake_v` + 실제 부품 + 꼬리 끝 탐촉자 (D안). 엘보 통과 미해결
- `front_v2`: 앞 모듈 v2 단독 (바퀴 유닛 3개 120° 간격, 직선 서스펜션 + 옴니휠, 삼각 외피). `cad_params.xacro`와 `meshes/`는 `docs/강민수/cad/v2/urdf_export.py`가 CAD에서 만든 파일. 혼자서는 뒤가 처지므로 직관 시험은 `PIPE_XACRO_ARGS='rear_support=true'` (시뮬 전용 임시 뒤 받침, 실물 설계 아님). **Gazebo 미실행** (2026-11-26, xacro 풀이와 값 검사만 함: `docs/강민수/cad/v2/urdf_check_results.txt`)
- `common/common.xacro`: 공통 매크로 (관성, 옴니휠 롤러, 시뮬 플러그인)

새 모델이 지켜야 할 토픽, 플러그인, `robot.yaml` 키는 [docs/02 차체 모델](../../../docs/02_차체_모델.md)을 보세요.
