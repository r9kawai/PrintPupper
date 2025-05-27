import math
from Config import Configuration

def compute_unparallel_link_knee(config, theta_leg, theta_knee_internal):
    """
    Parameters:
    -----------
    theta_leg : float
        L1（大腿リンク）の絶対角度 [rad]
    theta_knee_internal : float
        L1とL2の内角 [rad]

    Returns:
    --------
    theta_s3 : float
        サーボ3の必要角度 [rad]、構成不能な場合は NaN
    """
    # 非平行リンク機構パラメータ（必要に応じて外部から設定可能）
    L1 = config.LEG_L1
    L2 = config.LEG_L2
    L3_offset = config.LEG_UNPRALLEL_L3
    L4 = config.LEG_UNPRALLEL_L4
    L5 = config.LEG_UNPRALLEL_L5
    ofstX = config.UNPRALLEL_ofstX
    ofstY = config.UNPRALLEL_ofstY

    # L2方向は L1から見て (π - 内角) だけ反時計回り
    theta_L2 = theta_leg + (math.pi - theta_knee_internal)

    # 膝位置（L1終端）
    knee_x = L1 * math.cos(theta_leg)
    knee_y = L1 * math.sin(theta_leg)

    # ロッド接続点（L3）
    L3x = knee_x + L3_offset * math.cos(theta_L2)
    L3y = knee_y + L3_offset * math.sin(theta_L2)

    dx = L3x - ofstX
    dy = L3y - ofstY
    dist = math.hypot(dx, dy)

    EPS = 1e-6
    if dist > (L4 + L5 + EPS) or dist < abs(L4 - L5) - EPS:
        return float('nan')

    cos_beta = (L4**2 + dist**2 - L5**2) / (2 * L4 * dist)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)

    phi = math.atan2(dy, dx)
    theta_s3_a = phi - beta
    theta_s3_b = phi + beta

    def angle_diff(a, b):
        d = (a - b + math.pi) % (2 * math.pi) - math.pi
        return abs(d)

    return theta_s3_a if angle_diff(theta_s3_a, theta_L2) < angle_diff(theta_s3_b, theta_L2) else theta_s3_b

if __name__=="__main__":
    _config = Configuration()
    
    theta_leg = math.radians(135)
    theta_knee_internal = math.radians(-74.0815)
    
    theta_s3 = compute_unparallel_link_knee(_config, theta_leg, theta_knee_internal)
    print(f"compute_servo3_angle( {math.degrees(theta_leg):6.2f}, {math.degrees(theta_knee_internal):6.2f} ) = {math.degrees(theta_s3):6.4f}")

