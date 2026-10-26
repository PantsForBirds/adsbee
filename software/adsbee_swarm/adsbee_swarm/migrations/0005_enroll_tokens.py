"""
Enroll tokens get their own table: the current one (kept encrypted from now on) and the ones it
replaced. Each account's existing hash becomes its current token; it can't be shown (only the
hash was kept) until the owner makes a new one.
"""
import django.db.models.deletion
import django.utils.timezone
from django.db import migrations, models


def copy_hashes(apps, schema_editor):
    Account = apps.get_model('adsbee_swarm', 'Account')
    EnrollToken = apps.get_model('adsbee_swarm', 'EnrollToken')
    EnrollToken.objects.bulk_create([
        EnrollToken(account_id=a.pk, token_hash=a.enroll_token_hash, prefix=a.enroll_token_prefix or '',
                    created_at=a.enroll_token_rotated_at or a.created_at)
        for a in Account.objects.exclude(enroll_token_hash=None).exclude(enroll_token_hash='')])


def copy_back(apps, schema_editor):
    Account = apps.get_model('adsbee_swarm', 'Account')
    EnrollToken = apps.get_model('adsbee_swarm', 'EnrollToken')
    for row in EnrollToken.objects.filter(retired_at=None).order_by('created_at'):
        Account.objects.filter(pk=row.account_id).update(
            enroll_token_hash=row.token_hash, enroll_token_prefix=row.prefix, enroll_token_rotated_at=row.created_at)


class Migration(migrations.Migration):

    dependencies = [
        ('adsbee_swarm', '0004_remove_sample_range_map'),
    ]

    operations = [
        migrations.CreateModel(
            name='EnrollToken',
            fields=[
                ('id', models.BigAutoField(auto_created=True, primary_key=True, serialize=False, verbose_name='ID')),
                ('token_hash', models.CharField(max_length=64, unique=True)),
                ('prefix', models.CharField(max_length=8)),
                ('sealed', models.TextField(blank=True)),
                ('created_at', models.DateTimeField(default=django.utils.timezone.now)),
                ('retired_at', models.DateTimeField(blank=True, null=True)),
                ('cut_off', models.BooleanField(default=False)),
                ('account', models.ForeignKey(on_delete=django.db.models.deletion.CASCADE, related_name='enroll_tokens', to='adsbee_swarm.account')),
            ],
            options={
                'ordering': ['-created_at', '-pk'],
            },
        ),
        migrations.AddField(
            model_name='device',
            name='enroll_token',
            field=models.ForeignKey(blank=True, null=True, on_delete=django.db.models.deletion.SET_NULL, related_name='+', to='adsbee_swarm.enrolltoken'),
        ),
        migrations.RunPython(copy_hashes, copy_back),
        migrations.RemoveField(
            model_name='account',
            name='enroll_token_hash',
        ),
        migrations.RemoveField(
            model_name='account',
            name='enroll_token_prefix',
        ),
        migrations.RemoveField(
            model_name='account',
            name='enroll_token_rotated_at',
        ),
    ]
