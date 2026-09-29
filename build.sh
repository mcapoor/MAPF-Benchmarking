#!/bin/bash
#
# Build the solver libraries in libs/, each into its own build/ directory.
#
#   ./build.sh                       build every library
#   ./build.sh cbsh2 bcp2            build only these (libmrp, mcts, cbsh2, bcp2, reloc)
#   ./build.sh --clean [library...]  delete the build/ directories first
#
# A submodule that has not been cloned yet is initialized first. Submodules
# that are already checked out are left as they are (no `git submodule
# update`, which would move them off any local commits).
#
# Requirements:
#   libmrp  CMake, Boost, yaml-cpp
#   mcts    CMake, Boost, yaml-cpp
#   cbsh2   CMake, Boost
#   reloc   make, a C++11 compiler
#   bcp2    CMake 3.21+, a C++23 compiler, Gurobi (Boost is downloaded by its
#           CMake). Gurobi is found through $GUROBI_DIR, $GUROBI_HOME or the
#           gurobi_cl on the PATH; set GUROBI_DIR to the directory holding
#           include/ and lib/ if it is somewhere else. GCC 16 cannot compile
#           it, so Clang is used instead if it is installed.
# Set CC/CXX to choose the compiler and JOBS the number of parallel jobs
# (default: all cores; lower it if the bcp2 build runs out of memory).
# On the Oscar cluster, `module load boost yaml-cpp gurobi` first. The solvers
# must be built on the machine that runs them.
#
# Every library is attempted even if an earlier one fails; the exit status is
# non-zero if any failed.

ROOT_DIR=$(dirname "$(readlink -f "$0")")
LIBS_DIR="$ROOT_DIR/libs"
JOBS=${JOBS:-$(nproc 2>/dev/null || echo 4)}
ALL_LIBS=(libmrp mcts cbsh2 bcp2 reloc)

# directory of each library in libs/ (all submodules except mcts_nonoverlap)
declare -A DIRS=(
    [libmrp]=libMultiRobotPlanning
    [mcts]=mcts_nonoverlap
    [cbsh2]=CBSH2-RTC-CHBP
    [bcp2]=bcp2-mapf
    [reloc]=reLOC
)

clean=0
libs=()
for arg in "$@"; do
    case "$arg" in
        --clean) clean=1 ;;
        -h|--help) sed -n '2,29s/^# \{0,1\}//p' "$0"; exit 0 ;;
        *)
            if [ -z "${DIRS[$arg]}" ]; then
                echo "unknown library '$arg' (expected one of: ${ALL_LIBS[*]})" >&2
                exit 2
            fi
            libs+=("$arg") ;;
    esac
done
[ ${#libs[@]} -eq 0 ] && libs=("${ALL_LIBS[@]}")


ensure_submodule() {
    local dir="$LIBS_DIR/$1"
    # a submodule that has not been cloned is an empty directory
    if [ -z "$(ls -A "$dir" 2>/dev/null)" ]; then
        echo "initializing submodule libs/$1"
        git -C "$ROOT_DIR" submodule update --init --recursive -- "libs/$1" || return 1
    fi
}

# The directory of the Gurobi installation (containing include/ and lib/)
gurobi_dir() {
    if [ -n "$GUROBI_DIR" ]; then
        echo "$GUROBI_DIR"
    elif [ -n "$GUROBI_HOME" ]; then
        echo "$GUROBI_HOME"
    elif command -v gurobi_cl >/dev/null; then
        dirname "$(dirname "$(readlink -f "$(command -v gurobi_cl)")")"
    fi
}

build_libmrp() {
    local dir="$LIBS_DIR/${DIRS[libmrp]}"
    cmake -S "$dir" -B "$dir/build" -DCMAKE_BUILD_TYPE=Release &&
    cmake --build "$dir/build" -j "$JOBS"
}

build_mcts() {
    local dir="$LIBS_DIR/${DIRS[mcts]}"
    cmake -S "$dir" -B "$dir/build" -DCMAKE_BUILD_TYPE=Release &&
    cmake --build "$dir/build" -j "$JOBS"
}

build_cbsh2() {
    local dir="$LIBS_DIR/${DIRS[cbsh2]}"
    # its CMakeLists.txt declares a minimum version that CMake 4 no longer accepts
    cmake -S "$dir" -B "$dir/build" -DCMAKE_BUILD_TYPE=RELEASE -DCMAKE_POLICY_VERSION_MINIMUM=3.5 &&
    cmake --build "$dir/build" -j "$JOBS"
}

# bcp2-mapf's CMakeLists.txt downloads some dependencies with FetchContent as a
# shallow clone of an abbreviated commit hash, which git cannot check out
# ("invalid reference"). Clone each of those in full into build/_deps-full/
# and print the -DFETCHCONTENT_SOURCE_DIR_<NAME> arguments that make CMake use
# them instead.
prefetch_bcp2_deps() {
    local dir="$1" name repo tag src
    awk '/FetchContent_Declare\(/ { name = $0; sub(/.*FetchContent_Declare\(/, "", name); sub(/[ \t)].*/, "", name) }
         /GIT_REPOSITORY/ { repo = $2 }
         /GIT_TAG/ { print name, repo, $2 }' "$dir/CMakeLists.txt" |
    while read -r name repo tag; do
        [[ "$tag" =~ ^[0-9a-f]{7,39}$ ]] || continue
        src="$dir/build/_deps-full/$name"
        if [ ! -d "$src/.git" ]; then
            git clone --quiet "$repo" "$src" >&2 || return 1
        fi
        git -C "$src" checkout --quiet "$tag" >&2 || return 1
        echo "-DFETCHCONTENT_SOURCE_DIR_${name^^}=$src"
    done
}

build_bcp2() {
    local dir="$LIBS_DIR/${DIRS[bcp2]}"
    local gurobi deps
    gurobi=$(gurobi_dir)
    if [ -z "$gurobi" ] || [ ! -f "$gurobi/include/gurobi_c.h" ]; then
        echo "Gurobi not found: set GUROBI_DIR to the directory with include/gurobi_c.h" >&2
        return 1
    fi
    echo "using Gurobi in $gurobi"
    local deps_args  # one argument per line (paths may contain spaces)
    deps_args=$(prefetch_bcp2_deps "$dir") || { echo "could not download the dependencies of bcp2-mapf" >&2; return 1; }
    deps=()
    [ -n "$deps_args" ] && mapfile -t deps <<< "$deps_args"
    # GCC 16 rejects its static_assert(std::is_trivial_v<...>) on classes with a
    # deleted default constructor, so use Clang there if no compiler was chosen
    local compiler=()
    local gcc_major
    gcc_major=$(g++ -dumpversion 2>/dev/null | cut -d. -f1)
    if [ -z "$CXX" ] && [ "${gcc_major:-0}" -ge 16 ] && command -v clang++ >/dev/null; then
        echo "g++ $gcc_major cannot compile bcp2-mapf, using clang++"
        compiler=(-DCMAKE_C_COMPILER=clang -DCMAKE_CXX_COMPILER=clang++)
    fi
    cmake -S "$dir" -B "$dir/build" -DCMAKE_BUILD_TYPE=Release "${compiler[@]}" \
          -DGUROBI_DIR="$gurobi" "${deps[@]}" &&
    cmake --build "$dir/build" --target bcp2-mapf -j "$JOBS"
}

build_reloc() {
    local dir="$LIBS_DIR/${DIRS[reloc]}"
    make -C "$dir" -j "$JOBS"
}


failed=()
for lib in "${libs[@]}"; do
    echo "=== building $lib (libs/${DIRS[$lib]}) ==="
    if ! ensure_submodule "${DIRS[$lib]}"; then
        failed+=("$lib")
        continue
    fi
    [ $clean -eq 1 ] && rm -rf "$LIBS_DIR/${DIRS[$lib]}/build"
    if ! "build_$lib"; then
        echo "*** $lib failed" >&2
        failed+=("$lib")
    fi
done

echo
if [ ${#failed[@]} -gt 0 ]; then
    echo "failed: ${failed[*]}" >&2
    exit 1
fi
echo "built: ${libs[*]}"
