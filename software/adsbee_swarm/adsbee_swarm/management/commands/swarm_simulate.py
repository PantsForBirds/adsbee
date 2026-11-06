"""Demo history: simulated bees on an account, one stored report per bee per hour, ending now."""
import random
from datetime import timedelta

from django.core.management.base import BaseCommand, CommandError
from django.utils import timezone

from adsbee_swarm import protocol
from adsbee_swarm.models import Account, Device
from adsbee_swarm.simulator import Bee


class Command(BaseCommand):
    help = 'Store simulated reports for an account (demos).'

    def add_arguments(self, parser):
        parser.add_argument('username')
        parser.add_argument('--bees', type=int, default=5)
        parser.add_argument('--hours', type=int, default=24 * 7)
        parser.add_argument('--seed', type=int, default=1)
        parser.add_argument('--legacy', action='store_true', help='one mixed range, as older firmware sends')
        parser.add_argument('--listen', action='store_true',
                            help='listen time, as newer firmware: every other bee listens to UAT and hears nothing, '
                                 'the third has Remote ID off')

    def handle(self, username, bees, hours, seed, legacy=False, listen=False, **options):
        account = Account.objects.filter(user__username=username).first()
        if account is None:
            raise CommandError(f'no account {username}')
        rng = random.Random(seed)
        now = timezone.now()
        for index in range(bees):
            bee = Bee.make(index, seed)
            device, _ = Device.objects.get_or_create(account=account, product=bee.product, unique_id=bee.unique_id)
            if not device.label:
                device.label = bee.label
                device.save(update_fields=['label'])
            kind = {}
            if listen:
                kind = {'listen': ('mode_s', 'uat') if index % 3 == 2 else ('mode_s', 'uat', 'remote_id'),
                        'quiet': ('uat',) if index % 2 else ()}
            for hour in range(hours, 0, -1):
                body = bee.report(rng=rng, protocols=not legacy, **kind)
                protocol.store(device, protocol.parse(body), now - timedelta(hours=hour - 1))
            self.stdout.write(f'{bee.label}: {bee.unique_id} {hours} h')
