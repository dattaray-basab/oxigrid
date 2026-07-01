#!/usr/bin/env bash
set -euo pipefail

README="README.md"
DOC_MAP="doc_map.md"
MARKER_START="<!-- DOC_MAP_START -->"
MARKER_END="<!-- DOC_MAP_END -->"

get_title() {
    # Extract the first ATX header from the file (e.g. '# Title')
    file="$1"
    fallback="$2"
    if header=$(grep -m1 -E '^[[:space:]]*#+' "$file" 2>/dev/null || true); then
        if [ -n "$header" ]; then
            # strip leading hashes and surrounding whitespace robustly
            printf '%s' "$(printf '%s' "$header" | sed 's/^[[:space:]]*#\+ *//; s/^[#[:space:]]*//; s/[[:space:]]*$//')"
            return
        fi
    fi
    # fallback: prettify the fallback name (module or filename)
    # replace -/_ with space and capitalize each word
    printf '%s' "$(printf '%s' "$fallback" | sed 's/[-_]/ /g' | awk '{for(i=1;i<=NF;i++){ $i=toupper(substr($i,1,1)) substr($i,2) }}1')"
}

generate_doc_map() {
    echo "# Module Documentation Map" > "$DOC_MAP"
    echo "" >> "$DOC_MAP"


    # Iterate each module directory under src/ and pick a representative markdown file.
    for module_dir in src/*; do
        [ -d "$module_dir" ] || continue
        module=$(basename "$module_dir")
        # prefer module/module.md
        module_doc="$module_dir/$module.md"
        if [ -f "$module_doc" ]; then
            title=$(get_title "$module_doc" "$module")
            echo "- [${title}](${module_doc})" >> "$DOC_MAP"
            continue
        fi
        # otherwise pick the first markdown file in the module directory
        first_md=$(find "$module_dir" -maxdepth 1 -type f -name '*.md' | sort | head -n1 || true)
        if [ -n "$first_md" ]; then
            title=$(get_title "$first_md" "$(basename "$first_md" .md)")
            echo "- [${title}](${first_md})" >> "$DOC_MAP"
            continue
        fi
        # if no markdown found, skip
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
