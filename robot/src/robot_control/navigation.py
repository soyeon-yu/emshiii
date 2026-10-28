FRONT_SAFE = 0.6
SIDE_SAFE = 0.05
DEAD_END_SIDE = 0.5   # 정면 막힘 + 좌우도 이 이하로 좁으면 막다른 길로 판단


def decide(front, left, right):
    if front <= FRONT_SAFE:
        if left < DEAD_END_SIDE and right < DEAD_END_SIDE:
            return "DEAD_END"
        return "OBSTACLE_FRONT"

    if left < SIDE_SAFE:
        return "OBSTACLE_LEFT"
    if right < SIDE_SAFE:
        return "OBSTACLE_RIGHT"

    return "FORWARD"

