#!/bin/bash

# Code Formatting Script for JSZR Navigation Module
# This script formats all C/C++ source files in specified directories using navigation/.clang-format
# Author: JSZR Development Team
# Usage: ./format_code.sh [directory_path1] [directory_path2] ... [options]

set -e  # Exit on any error

# Script configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NAVIGATION_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
CLANG_FORMAT_CONFIG="${NAVIGATION_ROOT}/.clang-format"

# Clang-format binary configuration
# Priority: 1. CLion bundled clang-format (v21+)  2. System clang-format
CLION_CLANG_FORMAT="$HOME/.local/share/JetBrains/Toolbox/apps/clion/bin/clang/linux/x64/bin/clang-format"
if [[ -f "$CLION_CLANG_FORMAT" ]]; then
    CLANG_FORMAT_BIN="$CLION_CLANG_FORMAT"
else
    CLANG_FORMAT_BIN="clang-format"
fi

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default configuration
DEFAULT_EXTENSIONS=("cpp" "cc" "c" "hpp" "h" "hxx" "cxx")
DIRECTORIES=()
DRY_RUN=false
RECURSIVE=true
VERBOSE=false
BACKUP=false

# Print usage information
usage() {
    echo -e "${BLUE}JSZR Code Formatter${NC}"
    echo ""
    echo -e "${YELLOW}Usage:${NC}"
    echo "  $(basename "$0") [OPTIONS] [DIRECTORIES...]"
    echo ""
    echo -e "${YELLOW}Description:${NC}"
    echo "  Formats all C/C++ source files in the specified directories using navigation/.clang-format"
    echo "  If no directories are specified, formats the current directory"
    echo ""
    echo -e "${YELLOW}Options:${NC}"
    echo "  -h, --help           Show this help message"
    echo "  -d, --dry-run        Show files that would be formatted without making changes"
    echo "  -v, --verbose        Enable verbose output"
    echo "  -b, --backup         Create backup files (.orig) before formatting"
    echo "  --no-recursive       Only format files in specified directories (not subdirectories)"
    echo "  --extensions EXT     Comma-separated list of file extensions (default: cpp,cc,c,hpp,h,hxx,cxx)"
    echo ""
    echo -e "${YELLOW}Examples:${NC}"
    echo "  $(basename "$0")                           # Format current directory"
    echo "  $(basename "$0") src/navigation            # Format navigation module"
    echo "  $(basename "$0") -d src/perception         # Dry run on perception module"
    echo "  $(basename "$0") -b --verbose src/slam     # Format with backup and verbose output"
    echo "  $(basename "$0") --extensions cpp,hpp src # Format only .cpp and .hpp files"
}

# Print error message and exit
error_exit() {
    echo -e "${RED}Error: $1${NC}" >&2
    exit 1
}

# Print info message
info() {
    echo -e "${BLUE}Info: $1${NC}"
}

# Print success message
success() {
    echo -e "${GREEN}$1${NC}"
}

# Print warning message
warning() {
    echo -e "${YELLOW}Warning: $1${NC}"
}

# Check if clang-format is available
check_clang_format() {
    # Check if the configured clang-format binary exists
    if [[ ! -f "$CLANG_FORMAT_BIN" ]] && ! command -v "$CLANG_FORMAT_BIN" &> /dev/null; then
        error_exit "clang-format is not installed. Please install it using: sudo apt-get install clang-format"
    fi

    if [[ $VERBOSE == true ]]; then
        info "Using clang-format: $CLANG_FORMAT_BIN"
        info "Version: $($CLANG_FORMAT_BIN --version)"
    fi
}

# Check if .clang-format exists
check_config() {
    if [[ ! -f "$CLANG_FORMAT_CONFIG" ]]; then
        error_exit "navigation/.clang-format config not found at $CLANG_FORMAT_CONFIG"
    fi
    
    if [[ $VERBOSE == true ]]; then
        info "Using navigation config: $CLANG_FORMAT_CONFIG"
    fi
}

# Build find command based on extensions
build_find_command() {
    local dir="$1"
    local find_cmd="find \"$dir\""
    
    if [[ $RECURSIVE == false ]]; then
        find_cmd="$find_cmd -maxdepth 1"
    fi
    
    find_cmd="$find_cmd -type f"
    
    # Add file extensions
    if [[ ${#DEFAULT_EXTENSIONS[@]} -eq 1 ]]; then
        find_cmd="$find_cmd -name \"*.${DEFAULT_EXTENSIONS[0]}\""
    else
        find_cmd="$find_cmd \\("
        for i in "${!DEFAULT_EXTENSIONS[@]}"; do
            if [[ $i -gt 0 ]]; then
                find_cmd="$find_cmd -o"
            fi
            find_cmd="$find_cmd -name \"*.${DEFAULT_EXTENSIONS[$i]}\""
        done
        find_cmd="$find_cmd \\)"
    fi
    
    echo "$find_cmd"
}

# Format a single file
format_file() {
    local file="$1"
    local file_relative="${file#$WORKSPACE_ROOT/}"
    
    if [[ $VERBOSE == true ]]; then
        info "Processing: $file_relative"
    fi
    
    if [[ $DRY_RUN == true ]]; then
        echo "Would format: $file_relative"
        return 0
    fi
    
    # Create backup if requested
    if [[ $BACKUP == true ]]; then
        cp "$file" "$file.orig"
        if [[ $VERBOSE == true ]]; then
            info "Created backup: $file_relative.orig"
        fi
    fi
    
    # Format the file
    if "$CLANG_FORMAT_BIN" -i --style=file:"$CLANG_FORMAT_CONFIG" "$file"; then
        if [[ $VERBOSE == true ]]; then
            success "Formatted: $file_relative"
        fi
        return 0
    else
        warning "Failed to format: $file_relative"
        return 1
    fi
}

# Process a directory
process_directory() {
    local dir="$1"
    
    if [[ ! -d "$dir" ]]; then
        warning "Directory not found: $dir"
        return 1
    fi
    
    info "Processing directory: $dir"
    
    local find_cmd=$(build_find_command "$dir")
    local file_count=0
    local success_count=0
    local error_count=0
    
    # Execute find command and process files
    while IFS= read -r -d '' file; do
        ((file_count++))
        if format_file "$file"; then
            ((success_count++))
        else
            ((error_count++))
        fi
    done < <(eval "$find_cmd -print0")
    
    if [[ $file_count -eq 0 ]]; then
        warning "No source files found in: $dir"
    else
        if [[ $DRY_RUN == true ]]; then
            info "Found $file_count source files in: $dir"
        else
            success "Processed $file_count files in: $dir (Success: $success_count, Errors: $error_count)"
        fi
    fi
    
    return $error_count
}

# Parse command line arguments
parse_args() {
    while [[ $# -gt 0 ]]; do
        case $1 in
            -d|--dry-run)
                DRY_RUN=true
                shift
                ;;
            -v|--verbose)
                VERBOSE=true
                shift
                ;;
            -b|--backup)
                BACKUP=true
                shift
                ;;
            --no-recursive)
                RECURSIVE=false
                shift
                ;;
            --extensions)
                if [[ -n "$2" && "$2" != -* ]]; then
                    IFS=',' read -r -a DEFAULT_EXTENSIONS <<< "$2"
                    shift 2
                else
                    error_exit "--extensions requires a comma-separated list of extensions"
                fi
                ;;
            -*)
                error_exit "Unknown option: $1"
                ;;
            *)
                DIRECTORIES+=("$1")
                shift
                ;;
        esac
    done
    
    # If no directories specified, use current directory
    if [[ ${#DIRECTORIES[@]} -eq 0 ]]; then
        DIRECTORIES=(".")
    fi
}

# Main function
main() {
    echo -e "${BLUE}=== JSZR Code Formatter ===${NC}"
    
    # Check for help option early
    for arg in "$@"; do
        if [[ "$arg" == "-h" || "$arg" == "--help" ]]; then
            usage
            exit 0
        fi
    done
    
    # Parse arguments
    parse_args "$@"
    
    # Validate environment
    check_clang_format
    check_config
    
    if [[ $DRY_RUN == true ]]; then
        info "DRY RUN MODE - No files will be modified"
    fi
    
    if [[ $VERBOSE == true ]]; then
        info "Extensions to process: ${DEFAULT_EXTENSIONS[*]}"
        info "Recursive: $RECURSIVE"
        info "Backup: $BACKUP"
    fi
    
    # Process each directory
    local total_errors=0
    for dir in "${DIRECTORIES[@]}"; do
        # Convert to absolute path if relative
        if [[ "$dir" != /* ]]; then
            dir="$(cd "$dir" 2>/dev/null && pwd)" || {
                warning "Cannot access directory: $dir"
                ((total_errors++))
                continue
            }
        fi
        
        if ! process_directory "$dir"; then
            ((total_errors++))
        fi
        echo ""  # Add spacing between directories
    done
    
    # Final summary
    if [[ $DRY_RUN == true ]]; then
        success "Dry run completed. Use without -d/--dry-run to apply formatting."
    elif [[ $total_errors -eq 0 ]]; then
        success "All formatting operations completed successfully!"
    else
        warning "Completed with $total_errors errors. Check the output above for details."
        exit 1
    fi
}

# Run main function with all arguments
main "$@"