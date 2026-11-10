"""Texture the colliding parcel surface in the browser, GL and RTX."""

from contextvars import ContextVar

import numpy as np

MAILER_CELLS = (12, 16)


def atlas_uvs(triangles, cells=MAILER_CELLS):
    columns, rows = cells
    count = (columns + 1) * (rows + 1)
    triangles = np.asarray(triangles, dtype=np.int32).reshape(-1, 3)
    surface = triangles.reshape(-1) % count
    printed = np.repeat(np.all(triangles >= count, axis=1), 3)
    return np.column_stack(
        (
            0.005 + 0.49 * (surface // (columns + 1)) / rows + 0.5 * ~printed,
            0.005 + 0.99 * (surface % (columns + 1)) / columns,
        )
    ).astype(np.float32)


def install():
    _install_browser()
    _install_gl()
    _install_rtx()


def _install_browser():
    from robosimstudio_studio.garments import ViewerStudio, glb_for

    if getattr(ViewerStudio, "_shipping_label_atlas", False):
        return
    original = ViewerStudio._draw_garment

    def draw_garment(self, garment, points, hidden):
        if garment.name != "shipping_mailer" or hidden or not garment.finish.texture:
            return original(self, garment, points, hidden)
        vertices, normals = garment.surface(points)
        corners = garment.triangles.reshape(-1)
        faces = np.arange(len(corners), dtype=np.int32).reshape(-1, 3)
        blob = glb_for(
            vertices[corners],
            faces,
            garment.color,
            garment.finish,
            uvs=atlas_uvs(garment.triangles, garment.grid_cells),
            normals=normals[corners],
            cache=False,
        )
        if blob is None:
            return original(self, garment, points, hidden)
        name = self._qualify(f"/model/garments/{garment.name}")
        handle = self._server.scene.add_glb(name, glb_data=blob, cast_shadow=True, receive_shadow=True)
        self._garment_nodes[name] = handle
        self._garment_hidden[name] = False

    ViewerStudio._draw_garment = draw_garment
    ViewerStudio._shipping_label_atlas = True


def _install_gl():
    import mujoco
    from PIL import Image

    from robosimstudio.core.render.finish import texture_path
    from robosimstudio.solvers.render.gl import RendererGL

    if getattr(RendererGL, "_shipping_label_atlas", False):
        return
    active = ContextVar("shipping_label_gl", default=None)
    original_init = RendererGL.__init__
    original_compile = mujoco.MjSpec.compile

    def compile_spec(spec, *args, **kwargs):
        context = active.get()
        if context is not None:
            renderer, handles, finishes = context
            for index, part in enumerate(renderer._drawables.meshes):
                finish = finishes.get(part.name)
                if part.name != "shipping_mailer" or finish is None or not finish.texture:
                    continue
                with Image.open(texture_path(finish.texture)) as image:
                    pixels = np.asarray(image.convert("RGB"), dtype=np.uint8)
                name = f"shipping_label{index}"
                texture = spec.add_texture(
                    name=name,
                    type=mujoco.mjtTexture.mjTEXTURE_2D,
                    width=pixels.shape[1],
                    height=pixels.shape[0],
                    nchannel=3,
                )
                texture.data = pixels.tobytes()
                spec.add_material(name=name, textures=["", name], specular=0.15, shininess=0.25)
                uvs = atlas_uvs(handles[part.name].triangles)
                uvs = uvs * np.asarray(finish.uv_scale) + np.asarray(finish.uv_offset)
                uvs[:, 1] = 1.0 - uvs[:, 1]
                indices = np.arange(len(uvs), dtype=np.int32).reshape(-1, 3)
                mesh = spec.mesh(f"deformable{index}")
                mesh.usertexcoord = uvs.ravel().tolist()
                mesh.userfacetexcoord = np.concatenate((indices, indices[:, ::-1])).ravel().tolist()
                spec.geom(f"deformable{index}").material = name
        return original_compile(spec, *args, **kwargs)

    def initialize(self, *args, deformable_finishes=None, **kwargs):
        token = active.set((self, kwargs.get("deformables") or {}, deformable_finishes or {}))
        try:
            original_init(self, *args, **kwargs)
        finally:
            active.reset(token)

    mujoco.MjSpec.compile = compile_spec
    RendererGL.__init__ = initialize
    RendererGL.wants = (*RendererGL.wants, "deformable_finishes")
    RendererGL._shipping_label_atlas = True


def _install_rtx():
    from robosimstudio.solvers.render import rtx

    if getattr(rtx, "_shipping_label_atlas_v3", False):
        return
    original = getattr(rtx, "_shipping_original_viewer_class", rtx._viewer_class)
    rtx._shipping_original_viewer_class = original

    def viewer_class():
        cls = original()
        original_cache = cls._build_deformable_cache

        def build_cache(self, state):
            import warp as wp

            cache = original_cache(self, state)
            entry = cache.get("shipping_mailer")
            if entry is not None:
                _, _, triangles, _, _ = entry
                particles = np.asarray(self._deformables["shipping_mailer"].particles, dtype=np.int32)
                vertices = triangles.reshape(-1)
                uvs = atlas_uvs(triangles)
                indices = np.arange(len(vertices), dtype=np.int32).reshape(-1, 3)
                cache["shipping_mailer"] = (
                    wp.array(particles[vertices], dtype=wp.int32, device=self.device),
                    wp.array(indices.reshape(-1), dtype=wp.int32, device=self.device),
                    indices,
                    wp.empty(len(vertices), dtype=wp.vec3, device=self.device),
                    wp.array(uvs, dtype=wp.vec2, device=self.device),
                )
            return cache

        cls._build_deformable_cache = build_cache
        return cls

    rtx._viewer_class = viewer_class
    rtx._shipping_label_atlas_v3 = True
