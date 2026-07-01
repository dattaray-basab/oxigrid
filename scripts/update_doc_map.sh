#!/usr/bin/env bash
set -euo pipefail

README="README.md"
DOC_MAP="doc_map.md"
MARKER_START="<!-- DOC_MAP_START -->"
MARKER_END="<!-- DOC_MAP_END -->"

title_for() {
    case "$1" in
        analytics) echo "Analytics" ;;
        battery) echo "Battery" ;;
        digitaltwin) echo "Digital Twin" ;;
        harmonics) echo "Harmonics" ;;
        io) echo "IO" ;;
        monitoring) echo "Monitoring" ;;
        network) echo "Network" ;;
        optimize) echo "Optimize" ;;
        planning) echo "Planning" ;;
        powerelectronics) echo "Power Electronics" ;;
        powerflow) echo "Power Flow" ;;
        powerquality) echo "Power Quality" ;;
        protection) echo "Protection" ;;
        renewable) echo "Renewable" ;;
        security) echo "Security" ;;
        simulation) echo "Simulation" ;;
        stability) echo "Stability" ;;
        testcases) echo "Testcases" ;;
        units) echo "Units" ;;
        *) printf "%s" "${1^}" ;;
    esac
}

generate_doc_map() {
    echo "# Module Documentation Map" > "$DOC_MAP"
    echo "" >> "$DOC_MAP"
    for d in src/*; do
        [ -d "$d" ] || continue
        base=$(basename "$d")
        doc="$d/${base}.md"
        if [ -f "$doc" ]; then
            title=$(title_for "$base")
            echo "- [${title}](${doc})" >> "$DOC_MAP"
        fi
    done
    echo "Generated $DOC_MAP"
}

update_readme() {
    if [ ! -f "$README" ]; then
        echo "$README not found" >&2
        exit 1
    fi
    if [ ! -f "$DOC_MAP" ]; then
        echo "$DOC_MAP not found" >&2
        exit 1
    fi

    perl -0777 -e '
        BEGIN{ local $/; open my $fh, "<", "doc_map.md" or die $!; $doc=<$fh>; }
        local $_ = do { local $/; open my $rh, "<", "README.md"; <$rh> };
        s/\Q<!-- DOC_MAP_START -->\E.*?\Q<!-- DOC_MAP_END -->\E/"<!-- DOC_MAP_START -->\n".$doc."\n<!-- DOC_MAP_END -->"/s;
        print $_;
    ' > README.tmp
    mv README.tmp README.md
    echo "Updated $README"
}

generate_doc_map
update_readme
