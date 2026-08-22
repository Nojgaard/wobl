import numpy as np
import numpy.typing as npt

r"""
Legs of wobl can be represented as a 4 link closed kinematic loop with an extended
link (de) that is parallel with (cd). The system is actuated by a servo,
working as a crank at vertex a.

This class converts height (distance of hip joint a to wheel axle e)
to crank angle (angle at a) and vice versa. 

b    a
\----\
 \    \
  \    \
   ---------- e
   c    d
"""


class LegKinematics:
    def __init__(
        self, leg_keypoints: dict[str, npt.NDArray], angle_range: tuple[float, float]
    ) -> None:
        a = leg_keypoints["hip"]
        b = leg_keypoints["anchor"]
        c = leg_keypoints["link"]
        d = leg_keypoints["knee"]
        e = leg_keypoints["wheel"]

        self._ab = np.linalg.norm(a - b)
        self._bc = np.linalg.norm(b - c)
        self._cd = np.linalg.norm(c - d)
        self._ad = np.linalg.norm(a - d)
        self._de = np.linalg.norm(d - e)
        self._angle_offset = 1.74833122

        self.angle_range = angle_range
        self.height_range = (
            self.to_height(angle_range[1]),
            self.to_height(angle_range[0]),
        )

    def _tri_edge_length(self, a, b, theta):
        return np.sqrt(a**2 + b**2 - 2 * a * b * np.cos(theta))

    def _tri_angle(self, a, b, c):
        return np.acos((a**2 + b**2 - c**2) / (2 * a * b))

    def to_height(self, angle):
        theta_a = self._angle_offset - angle

        ab = self._ab
        bc = self._bc
        cd = self._cd
        ad = self._ad
        de = self._de

        bd = self._tri_edge_length(ab, ad, theta_a)
        theta_b = self._tri_angle(bd, ad, ab)
        theta_c = self._tri_angle(bd, cd, bc)
        d = np.pi - theta_b - theta_c
        ae = self._tri_edge_length(ad, de, d)

        return ae

    def to_angle(self, height: float) -> float:
        height = float(np.clip(height, *self.height_range))
        ab = self._ab
        bc = self._bc
        cd = self._cd
        ad = self._ad
        de = self._de
        ae = height

        theta_f = self._tri_angle(ad, de, ae)
        ca = self._tri_edge_length(cd, ad, np.pi - theta_f)
        theta_c = self._tri_angle(ca, ab, bc)
        theta_d = self._tri_angle(ca, ad, cd)

        angle = self._angle_offset - theta_c - theta_d
        return angle


if __name__ == "__main__":
    from woblpy.sim.robot import Robot

    robot = Robot()
    kin = LegKinematics(robot.leg_keypoints, robot.servo_limits())

    print("Angle Range:", kin.angle_range)
    print("Nominal Height:", kin.to_height(0.1))
    print("Nominal Angle:", kin.to_angle(0.1431165062317346))
    print("Max Height:", kin.to_height(-0.20))
    print("Min Height:", kin.to_height(0.65))
