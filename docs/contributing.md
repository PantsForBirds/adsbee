# Contributing to these docs

These docs live in the [`docs/` folder of the ADSBee repository](https://github.com/PantsForBirds/adsbee/tree/main/docs) and are built with [MkDocs](https://www.mkdocs.org/) and [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/). Anyone can suggest a change with a pull request.

## Suggest a change from your browser

1. Click **Edit this page** (the pencil icon at the top right) to open the page's Markdown source on GitHub.
2. Fork the repository if GitHub asks, and edit the page.
3. Click **Commit changes**, create a new branch, and open a pull request against `main`.
4. Once a maintainer merges it, the `latest` docs rebuild automatically.

## Preview the docs locally

For bigger changes (new pages, images, navigation), preview the site on your computer. You need Python 3.

```bash
git clone https://github.com/PantsForBirds/adsbee.git
cd adsbee
pip install -r docs/requirements.txt
mkdocs serve
```

Open <http://127.0.0.1:8000/>; it reloads as you edit. Before opening a pull request, check that the site builds without warnings:

```bash
mkdocs build --strict
```

## Where things go

- Pages are Markdown files under `docs/`. Add new pages to the `nav` section of `mkdocs.yml`.
- Images go in the matching section folder under `docs/assets/`, linked with a relative path, e.g. `![Alt text](../assets/guides/my-image.png)`.
- Link to other pages by relative path to their Markdown files (`../guides/update-firmware.md`) so links work in every docs version.
- Topics already covered by a README in the repository (such as building the firmware) should link to it on GitHub instead of copying it.
- Use admonitions for notes and warnings:

    ```markdown
    !!! warning

        Text of the warning.
    ```

## Docs versions

The version selector at the top of the page switches between docs versions.

- `latest` is built from `main` whenever `docs/` or `mkdocs.yml` changes.
- Each final firmware release gets a version named after its tag: `adsbee_1090-0.9.1` becomes `1090-0.9.1`, `adsbee_1421-0.3.11` becomes `1421-0.3.11`. Release candidates (`-rc`) don't.

Publishing is done by `.github/workflows/docs.yml` with [mike](https://github.com/jimporter/mike).
