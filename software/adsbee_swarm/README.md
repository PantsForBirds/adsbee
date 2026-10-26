# adsbee_swarm

A swarm server for ADSBee receivers. Each bee reports hourly: message counts, range maps (Mode S,
UAT, Remote ID), its firmware. You get a map of your bees, how far each one hears per altitude and
protocol, and staged firmware targets. It is a reusable Django app (GPL-3.0) with a plain reference UI; adsbee.aero runs the same
app behind its own pages.

- Protocol and API: [docs/swarm_api.md](docs/swarm_api.md), [adsbee_swarm/openapi.yaml](adsbee_swarm/openapi.yaml)
- Simulator: `python -m adsbee_swarm.simulator --server <url> --token <enroll token>`

## Self-host

Needs Docker and a DNS name pointing at the host.

```sh
cd software/adsbee_swarm/docker
cp env.example .env               # set SWARM_DOMAIN, POSTGRES_PASSWORD, DJANGO_SECRET_KEY
docker compose up -d --build
docker compose exec app python manage.py swarm_account create me --password '<password>'
```

The last command prints your **enroll token**; the ⚷ page shows it again (👁). Sign in at
`https://<SWARM_DOMAIN>/`. Caddy gets the TLS certificate. `swarm_account` also does `grant`,
`revoke`, `disable`, `enable` and `rotate` (`--cut-off`: see below); `/admin/` works for a superuser
(`manage.py createsuperuser`).

Each bee gets an hourly and a daily row from its first report; windows over 3 days read the daily
rows. Retention runs daily in the `app` container (`manage.py swarm_retention`): hourly rows older
than 90 days are deleted, daily rows are kept. Change it with `ADSBEE_SWARM_HOURLY_RETENTION_DAYS`.

Updating: the container runs `migrate` on start. Migrations `0003_sample_range` and
`0004_remove_sample_range_map` move each stored range map to its own row as protocol `aircraft`
(older firmware's Mode S and UAT mixed); it stays under ✈ (all aircraft). Back up the database
first; going back past 0003 merges per-protocol maps into one and drops Remote ID.

Migration `0005_enroll_tokens` moves each account's enroll token to its own table. Bees keep
reporting. The token can't be shown again (only its hash was kept) until you make a new one.
The current token is kept encrypted with `ADSBEE_SWARM_TOKEN_KEY` (a Fernet key; unset: derived
from `DJANGO_SECRET_KEY`). Changing either key hides the token until the next new one; bees keep
reporting.

## Point a bee at it

In the bee's console:

```
AT+SWARM=1,https://<SWARM_DOMAIN>,<enroll token>
AT+SWARM?
```

The bee shows up after its first report, 2–5 minutes after boot. It installs rollout targets only
when `AT+AUTO_UPDATE=1`.

New token (⟳ on the ⚷ page): new bees need it; bees already on the account keep reporting with the
old one and show ⚷ in the list until they get the new line. Cut off all bees (✕ ⬡): every old token
stops working, and each bee reports again only after it gets the new line.

Without a `DEVICE_VERIFIER` (self-hosted) the server can't tell a bee from someone posting its
unique ID: whoever holds an old token can still report as a bee already on the account. Use
"Cut off all bees" when a token leaks.

## In another Django project

```python
INSTALLED_APPS += ['adsbee_swarm']
urlpatterns += [path('', include('adsbee_swarm.api_urls')),          # device + user API
                path('swarm/', include('adsbee_swarm.urls'))]         # reference UI (optional)
ADSBEE_SWARM_DEVICE_VERIFIER = 'myapp.keys.verify_swarm_report'      # (request, report, account) -> bool
# The verifier raises adsbee_swarm.VerifierUnavailable when it cannot decide (no keys): 503, Retry-After.
```

`adsbee_swarm.services` has what staff pages need: `create_account`, `grant`, `revoke`,
`set_disabled`, `rotate_enroll_token` (`cut_off=`), `reveal_enroll_token` (owner's session only),
`create_api_token`. Settings are listed in `adsbee_swarm/conf.py`; `ADSBEE_SWARM_AUDIT` names a
function called on enroll token events (never given the token).

## Develop

```sh
poetry install --with dev
poetry run pytest
poetry run python manage.py migrate && poetry run python manage.py createcachetable
poetry run python manage.py swarm_account create demo --password demo
poetry run python manage.py swarm_simulate demo --bees 5 --hours 168   # --legacy: one mixed range
DJANGO_DEBUG=1 poetry run python manage.py runserver
```
