from django.contrib import admin

from .models import Account, ApiToken, Device, EnrollToken, Rollout, Sample, SampleRange


class EnrollTokenInline(admin.TabularInline):
    """Which tokens exist; never the token itself (not even encrypted)."""
    model = EnrollToken
    fields = ('prefix', 'created_at', 'retired_at', 'cut_off')
    readonly_fields = fields
    extra = 0
    can_delete = False

    def has_add_permission(self, request, obj=None):
        return False


@admin.register(Account)
class AccountAdmin(admin.ModelAdmin):
    inlines = [EnrollTokenInline]
    list_display = ('user', 'privileges', 'disabled', 'enroll_token_prefix', 'created_at')
    list_filter = ('disabled',)
    search_fields = ('user__username', 'user__email')


@admin.register(ApiToken)
class ApiTokenAdmin(admin.ModelAdmin):
    list_display = ('prefix', 'name', 'account', 'created_at', 'last_used_at')
    readonly_fields = ('token_hash', 'prefix')


@admin.register(Device)
class DeviceAdmin(admin.ModelAdmin):
    list_display = ('unique_id', 'label', 'product', 'account', 'last_report_at', 'auto_update', 'removed_at')
    list_filter = ('product', 'auto_update')
    search_fields = ('unique_id', 'label', 'account__user__username')
    readonly_fields = ('enroll_token',)


@admin.register(Rollout)
class RolloutAdmin(admin.ModelAdmin):
    list_display = ('tag', 'product', 'account', 'scope', 'stage', 'paused', 'cancelled_at', 'created_at')
    filter_horizontal = ('devices', 'test_devices')


class SampleRangeInline(admin.TabularInline):
    model = SampleRange
    fields = ('protocol', 'unit_m')
    readonly_fields = ('protocol', 'unit_m')
    extra = 0


@admin.register(Sample)
class SampleAdmin(admin.ModelAdmin):
    inlines = [SampleRangeInline]
    list_display = ('device', 'start', 'span_s', 'reports', 'es_valid', 'aircraft_max')
    list_filter = ('span_s',)
