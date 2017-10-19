# Run this script with "bash create-migration.sh". Do not make it executable.

date=$(date +'%Y-%m-%d')
same_date_count=$(ls | grep "$date" | wc -l)
filename="${date}_$(printf "%03d" "$same_date_count")"

cat > "$filename" <<EOF
#!/bin/bash

# Write your script here. Current working directory is \$ROBAIR_HOME.
EOF

chmod +x "$filename"
