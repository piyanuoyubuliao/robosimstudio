"""Texture the existing colliding slab faces; no geometry or state changes."""

import numpy as np


def install():
    from robosimstudio.solvers.render import rtx

    if getattr(rtx, "_shipping_label_atlas_v2", False):
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
                count = len(particles) // 2
                vertices = triangles.reshape(-1)
                surface = vertices % count
                printed = np.repeat(np.all(triangles >= count, axis=1), 3)
                uvs = np.column_stack((
                    0.005 + 0.49 * (surface // 13) / 16 + 0.5 * ~printed,
                    0.005 + 0.99 * (surface % 13) / 12,
                )).astype(np.float32)
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
    rtx._shipping_label_atlas_v2 = True
