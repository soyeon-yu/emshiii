import time
import motor

STEP = 5
HOLD = 1.5   # 각 단계 유지 시간(초)

def run(side, sign):
    c = 0
    print(f"\n=== {side} 바퀴, 방향 {'앞' if sign > 0 else '뒤'} ===")
    print("바퀴가 '꾸준히' 돌기 시작하면 q + Enter. 계속하려면 그냥 Enter.")
    while c < 300:
        c += STEP
        # 매 단계마다 정지에서 시작해서 정지마찰을 이기는 값을 잰다
        motor.stop_robot()
        time.sleep(0.6)
        l, r = (c * sign, 0) if side == "왼쪽" else (0, c * sign)
        t0 = time.time()
        while time.time() - t0 < HOLD:
            motor.send_motor(l, r)
            time.sleep(0.1)
        motor.stop_robot()
        ans = input(f"CMD={c * sign} 이었음. 돌았으면 q, 아니면 Enter: ")
        if ans.strip().lower() == "q":
            print(f"--> {side} {'앞' if sign > 0 else '뒤'} 최소 CMD ≈ {c}")
            return c
    return None

try:
    res = {}
    res["왼쪽 앞"] = run("왼쪽", +1)
    res["오른쪽 앞"] = run("오른쪽", +1)
    print("\n결과:", res)
finally:
    motor.stop_robot()
    motor.close_motor()
