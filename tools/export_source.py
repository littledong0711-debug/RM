"""Export a reviewable source bundle; never upload or include local diagnostics."""

from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


def source_files(root: Path) -> list[Path]:
    """Use a whitelist so a new local folder is not published accidentally."""
    files = [root / name for name in (
        "README.md", "run_test.sh", ".gitignore", ".gitattributes",
        ".editorconfig", ".clang-format", "armor_aim.code-workspace",
    )]
    for folder in ("src", "scripts", "tools", "docs"):
        for path in sorted((root / folder).rglob("*")):
            relative = path.relative_to(root)
            if "local" in relative.parts or "__pycache__" in relative.parts:
                continue
            if path.is_symlink():
                raise ValueError(f"Refusing symlink: {relative}")
            if path.is_file() and path.suffix in {
                ".cpp", ".hpp", ".py", ".sh", ".xml", ".yaml", ".md", ".png"
            }:
                files.append(path)
            elif path.is_file() and path.name == "CMakeLists.txt":
                files.append(path)
    for path in files:
        if not path.is_file() or path.is_symlink():
            raise ValueError(f"Missing or unsafe file: {path}")
    # All twelve images must travel with the tests, not just the last image.
    for index in range(12):
        required = root / "src/armor_aim/test/data/recorded_frames" / f"frame_{index:02d}_RGBA.png"
        if required not in files:
            raise ValueError(f"Missing fixture: {required}")
    return sorted(set(files))


def main() -> None:
    root = Path(__file__).resolve().parents[1]
    files = source_files(root)
    destination = root / "dist/armor_aim_source.zip"
    destination.parent.mkdir(exist_ok=True)
    # Replaces only this reproducible export, never original source files.
    with ZipFile(destination, "w", compression=ZIP_DEFLATED) as archive:
        for path in files:
            archive.write(path, path.relative_to(root).as_posix())
    with ZipFile(destination) as archive:
        if archive.testzip() is not None:
            raise RuntimeError("ZIP verification failed")
    print(f"Exported {len(files)} files: {destination}")


if __name__ == "__main__":
    main()
