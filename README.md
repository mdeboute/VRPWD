# VRPWD

[Here](./projet.pdf) is the french subject of the La Poste VRP With Drones project.

You'll need to have Python 3.8+ installed.
Don't forget to do a `pip install -r requirements.txt` before running the code.

For the main program:

Type `python3 src/vrpwdSolver.py -h` for help.

For the benchmark (don't forget to do a `chmod +x benchmark.sh`):

Type ./benchmark.sh -h for help.

## Social Networks

* Notion [here](https://www.notion.so/astral-agency/7a1486aff5da4701940d0b423bcd0d48?v=c1946cbaf6884adaa18bbe71a7ccfa28)
* Overleaf [here](https://l.messenger.com/l.php?u=https%3A%2F%2Fwww.overleaf.com%2F7247433974xnjmmvhzkqjr&h=AT39ufLhpJ7YwqAEoy17tP6CHyWdVP04OskifxIjZ9HtbnHy20vQbn_LDfzb77Vj1WULdPhleb8o7u-tvfjc2s3SOwLTrcYbQ2WKL_SfrGcR3vRCU8gy3VYFH7WlVGtJiAlA9KMzug8)
* Discord [here](https://discord.gg/NDpJqBMm)

## Coding Rules

To ensure consistency throughout the source code, keep these rules in mind as you are working:

* All features or bug fixes **must be tested** by one or more specs (unit-tests).
* All public API methods **must be documented**.
* Don't push your work on the `master` branch.
* Don't push code that are not working, that are not tested or that are in comments.

### Commit Message Format

*This specification is inspired by and supersedes the [AngularJS commit message format](https://docs.google.com/document/d/1QrDFcIiPjSLDn3EL15IJygNPiHORgU1_OOAqWjiDU5Y/edit#).*

We have very precise rules over how our Git commit messages must be formatted.
This format leads to **easier to read commit history**.

Each commit message consists of a **header**.

The `header` must conform to the [Commit Message Header](#commit-header) format below.

Any line of the commit message cannot be longer than 100 characters.

#### <a name="commit-header"></a> Commit Message Header

```txt
<type>(<scope>): <short summary>
  │       │             │
  │       │             └─⫸ Summary in present tense. Not capitalized. No period at the end.
  │       │
  │       └─⫸ Commit Scope: db|algo||instances|tests|utils|...
  │
  └─⫸ Commit Type: docs|feat|fix|perf|refactor|test|...
```

The `<type>` and `<summary>` fields are mandatory, the `(<scope>)` field is optional.
