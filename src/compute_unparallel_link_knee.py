import math
from Config import Configuration

# 非平行リンク機構の計算をする、この計算内部はpupperのコアと表現型が異なるため入出力の取り扱いに注意
class compute_unparallel_link_knee:
    def __init__(self, config):
        # 非平行リンク機構パラメータ
        self.L1 = config.LEG_L1
        self.L2 = config.LEG_L2
        self.L3_offset = config.LEG_UNPRALLEL_L3
        self.L4 = config.LEG_UNPRALLEL_L4
        self.L5 = config.LEG_UNPRALLEL_L5
        self.ofstX = config.UNPRALLEL_ofstX
        self.ofstY = config.UNPRALLEL_ofstY
        return

    def compute(self, theta_leg, theta_knee, mirror):
        # ミラー脚(右)なら角度を正負反転
        if mirror:
            theta_leg = -theta_leg
            theta_knee = -theta_knee

        # L1の終端座標（原点からのベクトル）
        x1 = self.L1 * math.cos(theta_leg)
        y1 = self.L1 * math.sin(theta_leg)

        # L2方向（L1から見て π - 内角だけ反時計回り）
        theta_L2 = theta_leg + (math.pi - theta_knee)

        # L2上のロッド接続点の位置（L3_offsetで任意）
        Px = x1 + self.L3_offset * math.cos(theta_L2)
        Py = y1 + self.L3_offset * math.sin(theta_L2)

        # サーボ3の位置（オフセット）
        x3 = self.ofstX
        y3 = self.ofstY

        # 求めたい角度 theta_s3 は、ロッド(L5)→リンク(L4)→原点
        # 三角形：P - サーボ3 - L4端点（この角度を求める）
        dx = Px - x3
        dy = Py - y3
        D = math.hypot(dx, dy)

        # 三角形の余弦定理
        cos_angle = (self.L4**2 + D**2 - self.L5**2) / (2 * self.L4 * D)
        if abs(cos_angle) > 1.0:
            return float('nan')  # 解なし

        angle_at_servo = math.acos(cos_angle)
        theta_line = math.atan2(dy, dx)

        # 2つあるうちの「L2方向に近い解」を選択
        theta_candidate1 = theta_line + angle_at_servo
        theta_candidate2 = theta_line - angle_at_servo

        # サーボホーン角度を選ぶ
        # 比較対象ベクトルは (cos(theta_L2), sin(theta_L2))
        v1 = (math.cos(theta_candidate1), math.sin(theta_candidate1))
        v2 = (math.cos(theta_candidate2), math.sin(theta_candidate2))
        target = (math.cos(theta_L2), math.sin(theta_L2))

        def dot(u, v): return u[0]*v[0] + u[1]*v[1]

        if dot(v1, target) > dot(v2, target):
            theta_s3 = theta_candidate1
        else:
            theta_s3 = theta_candidate2

        # ミラー脚(右)なら出力も反転
        if mirror:
            theta_s3 = -theta_s3

        return theta_s3





# 単体テスト、検算
# Unit testing, verification
if __name__=="__main__":
    _config = Configuration()
    culk = compute_unparallel_link_knee(_config)

    arg_leg_deg = 135
    arg_leg = math.radians(arg_leg_deg)
    arg_knee_deg = -89.9465
    arg_knee = math.radians(arg_knee_deg)
    arg_mirror = False
    theta_s3 = culk.compute(arg_leg, arg_knee, arg_mirror)
    print(f"compute_unparallel_link_knee( {arg_leg_deg}, {arg_knee_deg}, {arg_mirror}) = {math.degrees(theta_s3):6.4f}")

    arg_leg_deg = 126.87
    arg_leg = math.radians(arg_leg_deg)
    arg_knee_deg = -90
    arg_knee = math.radians(arg_knee_deg)
    arg_mirror = False
    theta_s3 = culk.compute(arg_leg, arg_knee, arg_mirror)
    print(f"compute_unparallel_link_knee( {arg_leg_deg}, {arg_knee_deg}, {arg_mirror}) = {math.degrees(theta_s3):6.4f}")

    arg_leg_deg = 143.13
    arg_leg = math.radians(arg_leg_deg)
    arg_knee_deg = -90
    arg_knee = math.radians(arg_knee_deg)
    arg_mirror = False
    theta_s3 = culk.compute(arg_leg, arg_knee, arg_mirror)
    print(f"compute_unparallel_link_knee( {arg_leg_deg}, {arg_knee_deg}, {arg_mirror}) = {math.degrees(theta_s3):6.4f}")

    arg_leg_deg = 151.997
    arg_leg = math.radians(arg_leg_deg)
    arg_knee_deg = -74.9299
    arg_knee = math.radians(arg_knee_deg)
    arg_mirror = False
    theta_s3 = culk.compute(arg_leg, arg_knee, arg_mirror)
    print(f"compute_unparallel_link_knee( {arg_leg_deg}, {arg_knee_deg}, {arg_mirror}) = {math.degrees(theta_s3):6.4f}")

    arg_leg_deg = -151.997
    arg_leg = math.radians(arg_leg_deg)
    arg_knee_deg = +74.9299
    arg_knee = math.radians(arg_knee_deg)
    arg_mirror = True
    theta_s3 = culk.compute(arg_leg, arg_knee, arg_mirror)
    print(f"compute_unparallel_link_knee( {arg_leg_deg}, {arg_knee_deg}, {arg_mirror}) = {math.degrees(theta_s3):6.4f}")

    arg_leg_deg = -126.65
    arg_leg = math.radians(arg_leg_deg)
    arg_knee_deg = 117.56
    arg_knee = math.radians(arg_knee_deg)
    arg_mirror = True
    theta_s3 = culk.compute(arg_leg, arg_knee, arg_mirror)
    print(f"compute_unparallel_link_knee( {arg_leg_deg}, {arg_knee_deg}, {arg_mirror}) = {math.degrees(theta_s3):6.4f}")
