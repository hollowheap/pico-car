#!/nix/store/9ipfvwnqp1q8ijnmi5sxvlx9r8w34lw3-bash-5.3p15/bin/bash
# Little script to find possibly-misnamed CONFIG lines
for CONF in PICO_CONFIG PICO_CMAKE_CONFIG PICO_BUILD_DEFINE; do
  for DEF in $(git grep "$CONF:" | sed -n "s/.*$CONF: \+\(\w\+\).*/\1/p"); do
    if [ $(git grep "$DEF" | wc -l) -lt 2 ]; then
        echo "$CONF: $DEF only appears once"
    fi
  done
done
