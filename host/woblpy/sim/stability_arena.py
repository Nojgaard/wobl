from dm_control import composer, mjcf


class StabilityArena(composer.Arena):
    def _build(self):
        self._mjcf_model = mjcf.RootElement(model="stability_arena")

        worldbody = self._mjcf_model.worldbody

        self._mjcf_model.visual.headlight.set_attributes(
            ambient=[0.4, 0.4, 0.4],
            diffuse=[0.8, 0.8, 0.8],
            specular=[0.1, 0.1, 0.1],
        )

        # A wedge gives the ramp a continuous ground-contacting lower edge.
        ramp_mesh = self._mjcf_model.asset.add(
            "mesh",
            name="ramp_mesh",
            vertex=[
                -0.20,
                0.45,
                0.0,
                0.20,
                0.45,
                0.0,
                -0.20,
                1.05,
                0.0,
                0.20,
                1.05,
                0.0,
                -0.20,
                0.45,
                0.002,
                0.20,
                0.45,
                0.002,
                -0.20,
                1.05,
                0.08,
                0.20,
                1.05,
                0.08,
            ],
            face=[
                0,
                3,
                1,
                0,
                2,
                3,
                0,
                6,
                2,
                0,
                4,
                6,
                0,
                5,
                4,
                0,
                1,
                5,
                1,
                7,
                5,
                1,
                3,
                7,
                2,
                7,
                3,
                2,
                6,
                7,
                4,
                7,
                6,
                4,
                5,
                7,
            ],
        )

        ramp_material = self._mjcf_model.asset.add(
            "material",
            name="ramp_material",
            rgba=[0.8, 0.45, 0.15, 1],
        )

        worldbody.add(
            "geom",
            name="ramp_up",
            type="mesh",
            mesh=ramp_mesh,
            material=ramp_material,
            friction=[0.8, 0.005, 0.0001],
        )

        # Low, partially embedded bumps form a short rough-terrain section.
        worldbody.add(
            "geom",
            name="bump_1",
            type="sphere",
            pos=[-0.11, 1.35, 0.018],
            size=[0.018],
            friction=[0.8, 0.005, 0.0001],
            rgba=[0.15, 0.45, 0.8, 1],
        )

        worldbody.add(
            "geom",
            name="bump_2",
            type="sphere",
            pos=[0.10, 1.52, 0.022],
            size=[0.022],
            friction=[0.8, 0.005, 0.0001],
            rgba=[0.15, 0.45, 0.8, 1],
        )

        worldbody.add(
            "geom",
            name="bump_3",
            type="sphere",
            pos=[-0.04, 1.72, 0.015],
            size=[0.015],
            friction=[0.8, 0.005, 0.0001],
            rgba=[0.15, 0.45, 0.8, 1],
        )

        worldbody.add(
            "geom",
            name="bump_4",
            type="sphere",
            pos=[0.13, 1.91, 0.020],
            size=[0.020],
            friction=[0.8, 0.005, 0.0001],
            rgba=[0.15, 0.45, 0.8, 1],
        )

        worldbody.add(
            "geom",
            name="bump_5",
            type="sphere",
            pos=[-0.12, 2.10, 0.017],
            size=[0.017],
            friction=[0.8, 0.005, 0.0001],
            rgba=[0.15, 0.45, 0.8, 1],
        )

        ground_texture = self._mjcf_model.asset.add(
            "texture",
            name="ground_texture",
            type="2d",
            builtin="checker",
            width=200,
            height=200,
            rgb1=[0.2, 0.3, 0.4],
            rgb2=[0.1, 0.2, 0.3],
            mark="edge",
            markrgb=[0.8, 0.8, 0.8],
        )

        ground_material = self._mjcf_model.asset.add(
            "material",
            name="ground_material",
            texture=ground_texture,
            texrepeat=[2, 2],
            texuniform=True,
            reflectance=0.0,
        )

        worldbody.add(
            "geom",
            name="ground",
            type="plane",
            size=[10, 10, 0.1],
            material=ground_material,
            friction=[0.8, 0.005, 0.0001],
        )

    @property
    def mjcf_model(self):
        return self._mjcf_model
