"""The device and user APIs only: `path('', include('adsbee_swarm.api_urls'))` at the site root."""
from django.urls import path

from . import api_device, api_user

app_name = 'adsbee_swarm_api'

urlpatterns = [
    path('api/swarm/v1/report', api_device.report, name='report'),
    path('api/swarm/v1/account', api_user.account, name='account'),
    path('api/swarm/v1/account/enroll-token', api_user.rotate_enroll_token, name='enroll_token'),
    path('api/swarm/v1/account/enroll-token/reveal', api_user.reveal_enroll_token, name='enroll_token_reveal'),
    path('api/swarm/v1/account/api-tokens', api_user.api_tokens, name='api_tokens'),
    path('api/swarm/v1/account/api-tokens/<int:pk>', api_user.api_token, name='api_token'),
    path('api/swarm/v1/devices', api_user.devices, name='devices'),
    path('api/swarm/v1/devices/<int:pk>', api_user.device, name='device'),
    path('api/swarm/v1/devices/<int:pk>/restore', api_user.device_restore, name='device_restore'),
    path('api/swarm/v1/range', api_user.range_map, name='range'),
    path('api/swarm/v1/stats', api_user.stats, name='stats'),
    path('api/swarm/v1/rollouts', api_user.rollout_list, name='rollouts'),
    path('api/swarm/v1/rollouts/<int:pk>', api_user.rollout_detail, name='rollout'),
]
