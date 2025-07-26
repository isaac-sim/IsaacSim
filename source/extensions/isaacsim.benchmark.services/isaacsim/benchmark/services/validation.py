import os
import shutil
import tempfile
import time

# Utility constants
DEFAULT_BLUR = 3  # Gaussian-blur kernel (pixels); 0 disables blurring
DEFAULT_TOLERANCE = 5.0  # Mean-difference threshold (0-255 scale)


class Validator:
    """Utility class for capturing and validating render-product images in benchmarks.

    Example usage:

        from isaacsim.benchmark.services.validation import Validator
        from pxr import Usd
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        validator = Validator(tolerance=2.5, blur_kernel=5)

        # Discover enabled render-products under the default *HydraTextures* root
        validator.build_render_product_map(stage)

        # Capture RGB images and (optionally) regenerate the golden set
        out_dir = validator.capture_images(
            stage,
            benchmark_name="my_benchmark",
            output_root="captures",
            golden_root="golden_data",
        )

        # Compare the new images against the reference
        validator.validate_images(out_dir, os.path.join("golden_data", "my_benchmark"))
    """

    def __init__(
        self,
        *,
        tolerance: float = DEFAULT_TOLERANCE,
        blur_kernel: int = DEFAULT_BLUR,
        regenerate_golden: bool = False,
        output_root: str = "captures",
        golden_root: str = "golden_data",
        auto_cleanup: bool = True,
    ) -> None:
        """Create a new *Validator* instance.

        Args:
            param tolerance: Mean-difference threshold used during validation.
            param blur_kernel: Gaussian-blur kernel size (0 disables blurring).
            param regenerate_golden: Replace the golden reference images instead of
                validating against them.
            param output_root: Base directory where capture folders are created.
            param golden_root: Base directory storing golden reference images.
            param auto_cleanup: If *True* the last capture folder is deleted
                automatically after :py:meth:`validate_images` completes.
        """
        self.tolerance: float = tolerance
        self.blur_kernel: int = blur_kernel
        self.regenerate_golden: bool = regenerate_golden

        self.output_root: str = os.fspath(output_root)
        self.golden_root: str = os.fspath(golden_root)

        self.auto_cleanup: bool = auto_cleanup

        self.render_product_map: dict[str, str] = {}
        self._last_capture_path: str | None = None
        self._last_benchmark_name: str | None = None

    def build_render_product_map(
        self,
        stage,
        root_prim_path: str = "/Render/OmniverseKit/HydraTextures",
    ) -> dict[str, str]:
        """Populate *self.render_product_map* by scanning *stage*.

        Every camera render product under *root_prim_path* is considered and validated via
        ``isaacsim.core.utils.render_product.get_camera_prim_path``.  Prims that
        fail the check are silently ignored.

        Args:
            param stage: USD stage to inspect.
            param root_prim_path: Path whose subtree is searched for render-products.

        Returns:
            Mapping from render-product USD path → human-readable folder name.
        """
        from isaacsim.core.utils.render_product import get_camera_prim_path
        from pxr import Usd

        root_prim = stage.GetPrimAtPath(root_prim_path)
        if not root_prim.IsValid():
            raise RuntimeError(f"Root prim '{root_prim_path}' not found on stage")

        self.render_product_map.clear()

        for prim in list(Usd.PrimRange(root_prim))[1:]:  # skip root itself
            try:
                nice_name = str(get_camera_prim_path(str(prim.GetPath()))).lstrip("/")
            except RuntimeError:
                continue  # not a render-product – ignore

            rp_path = str(prim.GetPath())
            if nice_name in self.render_product_map.values():
                base = nice_name
                idx = 1
                while f"{base}_{idx}" in self.render_product_map.values():
                    idx += 1
                nice_name = f"{base}_{idx}"

            self.render_product_map[rp_path] = nice_name

        return self.render_product_map

    def capture_images(
        self,
        stage,
        *,
        benchmark_name: str,
        output_root: str | None = None,
        golden_root: str | None = None,
        writer_name: str = "BasicWriter",
    ) -> str:
        """Capture RGB images for every discovered render-product.

        If *self.render_product_map* is empty it is automatically populated via
        :pymeth:`build_render_product_map` using the default *HydraTextures* root.

        The behaviour (overwrite vs. create timestamped directory) is controlled
        by *self.regenerate_golden*.

        Returns:
            Absolute path of the directory where PNGs were written.
        """
        import os
        from datetime import datetime

        import omni.replicator.core as rep

        if not self.render_product_map:
            self.build_render_product_map(stage)

        # Resolve roots
        if output_root is None:
            output_root = self.output_root
        else:
            self.output_root = os.fspath(output_root)

        if golden_root is None:
            golden_root = self.golden_root
        else:
            self.golden_root = os.fspath(golden_root)

        output_root = os.fspath(output_root)
        golden_root = os.fspath(golden_root)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

        if self.regenerate_golden:
            write_location = os.path.join(golden_root, benchmark_name)
            if os.path.isdir(write_location):
                shutil.rmtree(write_location)
        else:
            write_location = os.path.join(output_root, f"{benchmark_name}_{timestamp}")

        os.makedirs(write_location, exist_ok=True)

        writer = rep.WriterRegistry.get(writer_name)
        writer.initialize(output_dir=write_location, rgb=True)
        writer.attach(self.render_product_map.keys())
        rep.orchestrator.step()
        writer.detach()

        # give time for images to flush to disk
        time.sleep(1)

        # Rename folders to human-readable names
        for rp_path, nice_name in self.render_product_map.items():
            old_dir = os.path.join(write_location, rp_path.split("/")[-1])
            new_dir = os.path.join(write_location, nice_name)
            if os.path.isdir(old_dir):
                os.makedirs(os.path.dirname(new_dir), exist_ok=True)
                shutil.move(old_dir, new_dir)

        if self.regenerate_golden:
            print("\nGolden images regenerated – please verify visually:\n" + write_location)

        self._last_capture_path = write_location
        self._last_benchmark_name = benchmark_name

        return write_location

    def validate_images(
        self,
        captured_dir: str | None = None,
        golden_dir: str | None = None,
    ) -> bool:
        """Validate the captured images against the golden set.

        Args:
            captured_dir: Directory with newly captured PNGs.  If *None* the most
                recent directory returned by :py:meth:`capture_images` is used.
            golden_dir: Directory with golden PNGs.  If *None* the *golden_root*
                provided at construction time is used.
        """
        if captured_dir is None:
            captured_dir = self._last_capture_path
        else:
            self._last_capture_path = captured_dir

        if captured_dir is None:
            raise ValueError("captured_dir not specified and no capture has been performed yet")

        if golden_dir is None:
            if self._last_benchmark_name is not None:
                golden_dir = os.path.join(self.golden_root, self._last_benchmark_name)
            else:
                golden_dir = self.golden_root

        import cv2
        import numpy as np
        from PIL import Image

        print("\nValidating Images")
        print("-" * 40)

        def _collect_pngs(root: str) -> list[str]:
            return sorted(
                os.path.relpath(os.path.join(r, f), root)
                for r, _, files in os.walk(root)
                for f in files
                if f.endswith(".png")
            )

        all_passed = True

        for rp_path, nice_name in self.render_product_map.items():
            cap_root = os.path.join(captured_dir, nice_name)
            gold_root = os.path.join(golden_dir, nice_name)

            if not (os.path.isdir(cap_root) and os.path.isdir(gold_root)):
                print(f"Missing directories for render product '{nice_name}'")
                print(f"Capture dir: {cap_root}\nGolden dir:  {gold_root}")
                all_passed = False
                continue

            out_pngs = _collect_pngs(cap_root)
            gold_pngs = _collect_pngs(gold_root)

            common_files = sorted(set(out_pngs) & set(gold_pngs))

            if not common_files:
                print(f"No common PNG files for render product {nice_name}; skipping")
                continue

            for rel in common_files:
                cap_path = os.path.join(cap_root, rel)
                gold_path = os.path.join(gold_root, rel)

                cap_arr = np.array(Image.open(cap_path))
                gold_arr = np.array(Image.open(gold_path))

                if cap_arr.shape != gold_arr.shape:
                    print(f"Shape mismatch: {rel}")
                    all_passed = False
                    continue

                if self.blur_kernel > 0:
                    k = self.blur_kernel | 1  # must be odd
                    cap_arr = cv2.GaussianBlur(cap_arr, (k, k), 0)
                    gold_arr = cv2.GaussianBlur(gold_arr, (k, k), 0)

                diff = cv2.absdiff(cap_arr, gold_arr)
                mean_diff = diff.mean()

                if mean_diff <= self.tolerance:
                    continue  # passes – no diff written

                all_passed = False

                # ---------- visual diff ----------
                base = cap_arr.copy()
                base = self._ensure_rgb(base)

                mag = diff.max(axis=2) if diff.ndim == 3 else diff  # H×W
                norm = (mag * 255.0 / mag.max()).astype(np.uint8)

                red_layer = np.zeros_like(base)
                red_layer[..., 0] = 255  # R channel (RGB)

                alpha = (norm.astype(np.float32) / 255.0)[..., None]
                overlay = (red_layer.astype(np.float32) * alpha + base.astype(np.float32) * (1.0 - alpha)).astype(
                    np.uint8
                )

                tmp = tempfile.mkdtemp(prefix="diff_")
                out_png = os.path.join(tmp, f"diff_{os.path.basename(cap_path)}")
                self._write_png(out_png, overlay)
                print(f"[FAIL] {nice_name}: mean={mean_diff:.2f}  diff→ {out_png}")

        if not all_passed:
            print("\nValidation failed for some images. Check the diff images at the paths listed above.\n")
        else:
            print("\nValidation passed for all images.\n")

        if self.auto_cleanup:
            self.clear_last_capture()

        return all_passed

    def clear_last_capture(self) -> None:
        """Delete the last capture directory created by :pymeth:`capture_images`."""
        if self._last_capture_path and os.path.isdir(self._last_capture_path):
            shutil.rmtree(self._last_capture_path)
        self._last_capture_path = None

    @staticmethod
    def _ensure_rgb(arr):
        """Return a three-channel RGB *view* of *arr* (H×W×C)."""
        import cv2

        if arr.ndim == 2:  # Gray ⇒ RGB
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)
        if arr.shape[2] == 4:  # RGBA ⇒ RGB
            return arr[:, :, :3]
        return arr

    @staticmethod
    def _write_png(path: str, rgb) -> None:
        """Write *rgb* (RGB) to *path* as PNG using cv2 (expects BGR)."""
        import cv2

        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.imwrite(path, bgr)

    # e2e runner
    def run(
        self,
        stage,
        *,
        benchmark_name: str,
    ) -> bool:
        """Full pipeline: discover → capture → validate.

        Returns *True* if validation passes (or when *regenerate_golden* is
        *True*); *False* otherwise.
        """
        self.build_render_product_map(stage)
        self.capture_images(stage, benchmark_name=benchmark_name)

        if self.regenerate_golden:
            return True

        result = self.validate_images()

        return result

    @classmethod
    def from_cli_args(
        cls,
        args,
        *,
        auto_cleanup: bool | None = None,
    ) -> "Validator":
        """Construct a *Validator* directly from an *argparse* result object.

        The following attribute names are read if present: ``tolerance`` (DEFAULT_TOLERANCE), ``blur_kernel``
        (DEFAULT_BLUR), ``regenerate_golden`` (False), ``output_dir`` ("captures"),
        ``golden_dir`` ("golden_data").
        """
        tol = getattr(args, "tolerance", DEFAULT_TOLERANCE)
        blur = getattr(args, "blur_kernel", DEFAULT_BLUR)
        regen = getattr(args, "regenerate_golden", False)
        out_dir = getattr(args, "output_dir", "captures")
        gold_dir = getattr(args, "golden_dir", "golden_data")

        if auto_cleanup is None:
            auto_cleanup = True

        # Convert to absolute paths (relative → cwd)
        if not os.path.isabs(out_dir):
            out_dir = os.path.join(os.getcwd(), out_dir)
        if not os.path.isabs(gold_dir):
            gold_dir = os.path.join(os.getcwd(), gold_dir)

        return cls(
            tolerance=tol,
            blur_kernel=blur,
            regenerate_golden=regen,
            output_root=str(out_dir),
            golden_root=str(gold_dir),
            auto_cleanup=auto_cleanup,
        )
