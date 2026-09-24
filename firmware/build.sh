#!/bin/bash
# ADSBee firmware build dispatcher.
# Usage: ./build.sh <product> [args...]
#   product: adsbee_1090 | adsbee_1421
#   args are forwarded to firmware/<product>/build.sh, e.g.:
#     ./build.sh adsbee_1090            # all three 1090 targets (esp -> ti -> pico), Release
#     ./build.sh adsbee_1090 esp
#     ./build.sh adsbee_1090 -d test AircraftJSON
#     ./build.sh adsbee_1090 build_and_flash    # build everything, then flash an attached device
#     ./build.sh adsbee_1090 flash              # flash what is already built, no build steps
#     ./build.sh adsbee_1421            # CC1314 application, Release
#     ./build.sh adsbee_1421 programmer
#     ./build.sh adsbee_1421 clean ti
set -euo pipefail

script_dir="$(cd "$(dirname "$0")" && pwd)"
case "${1:-}" in
    adsbee_1090|adsbee_1421)
        product="$1"
        shift
        exec "$script_dir/$product/build.sh" "$@"
        ;;
    *)
        echo "Usage: $0 <adsbee_1090|adsbee_1421> [args...]" >&2
        echo "Run '$0 <product> --help' for product-specific targets." >&2
        exit 1
        ;;
esac
