"""Self-hosters' account admin: create, grant/revoke, disable/enable, rotate the enroll token
(a new one; bees on the account keep their old one, unless --cut-off)."""
from django.core.management.base import BaseCommand, CommandError

from adsbee_swarm import services
from adsbee_swarm.models import Account


class Command(BaseCommand):
    help = 'Manage swarm accounts.'

    def add_arguments(self, parser):
        parser.add_argument('action', choices=['create', 'grant', 'revoke', 'disable', 'enable', 'rotate'])
        parser.add_argument('username')
        parser.add_argument('--email', default='')
        parser.add_argument('--password', help='create: set a password (else none: the host invites)')
        parser.add_argument('--privilege', action='append', default=None)
        parser.add_argument('--cut-off', action='store_true',
                            help='rotate: every old enroll token stops working (each bee needs the new line)')

    def handle(self, action, username, email, password, privilege, cut_off=False, **options):
        try:
            if action == 'create':
                account, token = services.create_account(username, email=email, password=password,
                                                         privileges=privilege)
                self.stdout.write(f'created {account} privileges={account.privileges}\nenroll token: {token}')
                return
            account = Account.objects.filter(user__username=username).first()
            if account is None:
                raise CommandError(f'no account {username}')
            if action in ('grant', 'revoke'):
                for name in privilege or []:
                    getattr(services, action)(account, name)
                self.stdout.write(f'{account} privileges={account.privileges}')
            elif action in ('disable', 'enable'):
                services.set_disabled(account, action == 'disable')
                self.stdout.write(f'{account} {action}d')
            else:
                self.stdout.write(f'enroll token: {services.rotate_enroll_token(account, cut_off=cut_off)}')
                self.stdout.write('old tokens: cut off' if cut_off else 'old tokens: still work for bees already here')
        except services.SwarmError as exc:
            raise CommandError(str(exc)) from exc
