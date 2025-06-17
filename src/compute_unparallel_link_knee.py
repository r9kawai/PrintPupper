import math
from Config import Configuration

def compute_unparallel_link_knee(config, theta_leg, theta_knee, mirror_right):
    # 非平行リンク機構パラメータ
    L1 = config.LEG_L1
    L2 = config.LEG_L2
    L3_offset = config.LEG_UNPRALLEL_L3
    L4 = config.LEG_UNPRALLEL_L4
    L5 = config.LEG_UNPRALLEL_L5
    ofstX = config.UNPRALLEL_ofstX
    ofstY = config.UNPRALLEL_ofstY

    # 鏡像（左右）反転：theta_legとtheta_kneeを符号反転
    if mirror_right:
        theta_leg = -theta_leg
        theta_knee = -theta_knee

    # L1のベクトル
    P1_x = L1 * math.cos(theta_leg)
    P1_y = L1 * math.sin(theta_leg)

    # L2の方向（L1から見て反時計回りにtheta_knee）
    theta_L2 = theta_leg + theta_knee
    P2_x = P1_x + L2 * math.cos(theta_L2)
    P2_y = P1_y + L2 * math.sin(theta_L2)

    # L2の中点（M点）
    M_x = (P1_x + P2_x) / 2
    M_y = (P1_y + P2_y) / 2

    # サーボ3の原点（S3点）
    S3_x = ofstX
    S3_y = ofstY

    # ベクトルS3→Mの長さ
    dx = M_x - S3_x
    dy = M_y - S3_y
    d_sq = dx**2 + dy**2
    d = math.sqrt(d_sq)

    # 三角形 S3-L4-L5 の余弦定理による角度計算
    cos_a = (d_sq + L4**2 - L5**2) / (2 * d * L4)
    if abs(cos_a) > 1.0:
        return float('nan')  # 到達不能

    angle_a = math.acos(cos_a)

    # ベクトルS3→Mの角度
    angle_to_M = math.atan2(dy, dx)

    # S3から見たロッド（L4）の角度：反時計回りで算出
    theta_servo3_candidate = angle_to_M - angle_a

    # 解の選択：L2方向により近い解を選択
    theta_servo3_alt = angle_to_M + angle_a
    L2_vec = (math.cos(theta_L2), math.sin(theta_L2))
    diff1 = (math.cos(theta_servo3_candidate) - L2_vec[0])**2 + (math.sin(theta_servo3_candidate) - L2_vec[1])**2
    diff2 = (math.cos(theta_servo3_alt) - L2_vec[0])**2 + (math.sin(theta_servo3_alt) - L2_vec[1])**2
    theta_servo3 = theta_servo3_candidate if diff1 < diff2 else theta_servo3_alt

    # 結果反転
    if not mirror_right:
        theta_servo3 = -theta_servo3

    return theta_servo3





if __name__=="__main__":
    _config = Configuration()
    
    arg_leg_deg = 135
    arg_knee_deg = -89
    theta_leg = math.radians(arg_leg_deg)
    theta_knee_internal = math.pi - math.radians(arg_knee_deg)
    theta_s3 = compute_unparallel_link_knee(_config, theta_leg, theta_knee_internal, False)
    print(f"compute_unparallel_link_knee( {arg_leg_deg}, {arg_knee_deg} ) = {math.degrees(theta_s3):6.4f}")

    arg_leg_deg = -135
    arg_knee_deg = 89
    theta_leg = math.radians(arg_leg_deg)
    theta_knee_internal = math.pi - math.radians(arg_knee_deg)
    theta_s3 = compute_unparallel_link_knee(_config, theta_leg, theta_knee_internal, True)
    print(f"compute_unparallel_link_knee( {arg_leg_deg}, {arg_knee_deg} ) = {math.degrees(theta_s3):6.4f}")
