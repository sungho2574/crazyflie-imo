"""Blackbird 스타일 주기(periodic) 궤적 — 도형 정의 · 연속-랩 비행.

Antonini et al. *The Blackbird Dataset* 의 궤적 설계 철학(주기성 · 속도 격리 · yaw 모드)을
따르되, 데이터셋 없이 **도형을 코드로 직접 정의**한다(option a). 깔끔한 도형은 파라메트릭
수식(해석적 도함수)으로, 임의 도형은 주기 스플라인으로 만든다.
"""
