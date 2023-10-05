#/bin/bash

echo "Stressing CPU for 5 minutes..."
exec stress-ng -c $(nproc) -t 300
