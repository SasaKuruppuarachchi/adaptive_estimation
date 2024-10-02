# Pixi

install: https://pixi.sh/latest/basic_usage/

### Init (if not already done in the repo. I.e. if there's no `tpc` directory)

```bash
pixi init tpc
cd tpc
```

Activate env with: `pixi shell`

### Package installation
```bash
pixi add <package>
```
or edit the `pixi.toml` file directly
```
...
[dependencies]
numpy = ">=3.1.1,<3"
hydra-core = ">=1.3.2,<2"
python = ">=3.12.6,<4"
...

```

### Usage
Go wild

