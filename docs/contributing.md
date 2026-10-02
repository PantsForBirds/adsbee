# Contributing to these docs

These docs live in the [`docs/` folder of the ADSBee repository](https://github.com/PantsForBirds/adsbee/tree/main/docs) and are built with [MkDocs](https://www.mkdocs.org/) and the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme. Anyone can suggest a change with a pull request.

## Suggest a change from your browser

1. Open the page you want to change and click the **Edit this page** button (the pencil icon at the top right of the page). It opens the page's Markdown source on GitHub.
2. GitHub asks you to fork the repository if you don't have write access. Make your edits in the GitHub editor.
3. Click **Commit changes**, choose to create a new branch, and open a pull request against `main`.
4. A maintainer reviews the pull request. Once it is merged, the `latest` version of the docs is rebuilt automatically.

## Preview the docs locally

To work on bigger changes (new pages, images, navigation), clone the repository and preview the site on your computer. You need Python 3.

```bash
git clone https://github.com/PantsForBirds/adsbee.git
cd adsbee
pip install -r docs/requirements.txt
mkdocs serve
```

Then open <http://127.0.0.1:8000/> in a browser. The preview reloads as you edit files. Before opening a pull request, check that the site builds without warnings:

```bash
mkdocs build --strict
```

## Where things go

- Pages are Markdown files under `docs/`, and the navigation is the `nav` section of `mkdocs.yml` at the repository root. Add new pages there.
- Images go in `docs/assets/`, in the folder for the section they belong to. Link them with a relative path, for example `![Alt text](../assets/guides/my-image.png)`.
- Link to other pages with relative paths to their Markdown files (`../guides/update-firmware.md`), so the links keep working in every docs version.
- Topics that are already documented in a README in the repository (such as building the firmware) should link to that README on GitHub instead of copying it.
- Use admonitions for notes and warnings:

    ```markdown
    !!! warning

        Text of the warning.
    ```

## Docs versions

The site keeps a copy of the docs for each firmware release. The version selector at the top of the page switches between them.

- `latest` is built from the `main` branch every time `docs/` or `mkdocs.yml` changes.
- Each final firmware release gets its own version, named after its tag: `adsbee_1090-0.9.1` becomes `1090-0.9.1`, and `adsbee_1421-0.3.11` becomes `1421-0.3.11`. Release candidate tags (`-rc`) don't get a docs version.

Publishing is handled by the `docs` GitHub Actions workflow (`.github/workflows/docs.yml`) with [mike](https://github.com/jimporter/mike).
