"""A self-hostable site around adsbee_swarm. Everything set by environment (docker/compose.yml)."""
import os
from pathlib import Path

BASE_DIR = Path(__file__).resolve().parent.parent
SECRET_KEY = os.environ.get('DJANGO_SECRET_KEY', 'dev-only-not-secret')
DEBUG = os.environ.get('DJANGO_DEBUG', '') == '1'
ALLOWED_HOSTS = [h for h in os.environ.get('DJANGO_ALLOWED_HOSTS', 'localhost,127.0.0.1').split(',') if h]
CSRF_TRUSTED_ORIGINS = [o for o in os.environ.get('DJANGO_CSRF_TRUSTED_ORIGINS', '').split(',') if o]

INSTALLED_APPS = [
    'django.contrib.admin', 'django.contrib.auth', 'django.contrib.contenttypes', 'django.contrib.sessions',
    'django.contrib.messages', 'django.contrib.staticfiles', 'adsbee_swarm',
]
MIDDLEWARE = [
    'django.middleware.security.SecurityMiddleware', 'django.contrib.sessions.middleware.SessionMiddleware',
    'django.middleware.common.CommonMiddleware', 'django.middleware.csrf.CsrfViewMiddleware',
    'django.contrib.auth.middleware.AuthenticationMiddleware', 'django.contrib.messages.middleware.MessageMiddleware',
    'django.middleware.clickjacking.XFrameOptionsMiddleware',
]
ROOT_URLCONF = 'example_site.urls'
TEMPLATES = [{
    'BACKEND': 'django.template.backends.django.DjangoTemplates', 'DIRS': [], 'APP_DIRS': True,
    'OPTIONS': {'context_processors': ['django.template.context_processors.request',
                                       'django.contrib.auth.context_processors.auth',
                                       'django.contrib.messages.context_processors.messages']},
}]
WSGI_APPLICATION = 'example_site.wsgi.application'

if os.environ.get('POSTGRES_HOST'):
    DATABASES = {'default': {
        'ENGINE': 'django.db.backends.postgresql', 'HOST': os.environ['POSTGRES_HOST'],
        'PORT': os.environ.get('POSTGRES_PORT', '5432'), 'NAME': os.environ.get('POSTGRES_DB', 'swarm'),
        'USER': os.environ.get('POSTGRES_USER', 'swarm'), 'PASSWORD': os.environ.get('POSTGRES_PASSWORD', ''),
    }}
else:
    DATABASES = {'default': {'ENGINE': 'django.db.backends.sqlite3',
                             'NAME': os.environ.get('SQLITE_PATH', BASE_DIR / 'swarm.sqlite3')}}
# Rate limits must count across workers: the database cache (manage.py createcachetable).
CACHES = {'default': {'BACKEND': 'django.core.cache.backends.db.DatabaseCache', 'LOCATION': 'swarm_cache'}}

LOGIN_URL = 'login'
LOGIN_REDIRECT_URL = 'adsbee_swarm:devices'
LOGOUT_REDIRECT_URL = 'login'
STATIC_URL = '/static/'
STATIC_ROOT = os.environ.get('STATIC_ROOT', BASE_DIR / 'staticfiles')
USE_TZ = True
TIME_ZONE = 'UTC'
DEFAULT_AUTO_FIELD = 'django.db.models.BigAutoField'
SESSION_COOKIE_SECURE = CSRF_COOKIE_SECURE = not DEBUG and os.environ.get('DJANGO_INSECURE_COOKIES') != '1'
SECURE_PROXY_SSL_HEADER = ('HTTP_X_FORWARDED_PROTO', 'https')
ADSBEE_SWARM_CLIENT_IP_HEADER = os.environ.get('ADSBEE_SWARM_CLIENT_IP_HEADER') or None
ADSBEE_SWARM_TOKEN_KEY = os.environ.get('ADSBEE_SWARM_TOKEN_KEY') or None
LOGGING = {'version': 1, 'handlers': {'console': {'class': 'logging.StreamHandler'}},
           'root': {'handlers': ['console'], 'level': 'INFO'}}
