#!/bin/sh

build_start_time()
{
   ncolors=$(tput colors 2>/dev/null)
   if [ -n "$ncolors" ] && [ $ncolors -ge 8 ]; then
       color_red=$'\E'"[0;31m"
       color_green=$'\E'"[0;32m"
       color_yellow=$'\E'"[0;33m"
       color_white=$'\E'"[00m"
   else
       color_red=""
       color_green=""
       color_yellow=""
       color_white=""
   fi

   start_time=$(date +"%s")
   start_date=$(date +"%Y_%m_%d %H:%M:%S")
   echo -n "${color_green}"
   echo "start date:$start_date"
   echo -n "${color_white}"
   echo
   echo
   echo " ${color_white}"
   echo
}
build_end_time()
{
   end_time=$(date +"%s")
   end_date=$(date +"%Y_%m_%d %H:%M:%S")
   tdiff=$(($end_time-$start_time))
   hours=$(($tdiff / 3600 ))
   mins=$((($tdiff % 3600) / 60))
   secs=$(($tdiff % 60))

   echo
   echo -n "${color_green}"
   echo "start date:$start_date"
   echo "end date:$end_date"
   echo "build total time:"
   if [ $hours -gt 0 ] ; then
       printf "(%02g:%02g:%02g (hh:mm:ss))" $hours $mins $secs
   elif [ $mins -gt 0 ] ; then
       printf "(%02g:%02g (mm:ss))" $mins $secs
   elif [ $secs -gt 0 ] ; then
       printf "(%s seconds)" $secs
   fi
   echo
   echo " ${color_white}"
   echo
}
print_platform()
{
    echo
    echo "select platform:"
    echo "   1.  waipio"
    echo "   2.  kalama"
    echo "   3.  reserve"
    echo
}

choose_platform()
{
    local default_value=waipio
    local ANSWER
    print_platform
    echo "New Platform need add to here "
    echo -n "Which would you like? [$default_value]"

    if [[ -n "$1" ]] ; then
        ANSWER=$1
    else
        #read ANSWER
        ANSWER=$default_value
    fi

    echo "$ANSWER"
    echo

    case $ANSWER in
        1)
            variants_platform=waipio
            pre_path=waipio
        ;;
        waipio)
            variants_platform=waipio
            pre_path=waipio
        ;;
		2)
            variants_platform=kalama
            pre_path=kernel
        ;;
		kalama)
            variants_platform=kalama
            pre_path=kernel
        ;;
        parrot)
            variants_platform=parrot
        ;;
       *)
            variants_platform=waipio
            pre_path=waipio
        ;;
    esac
    echo "now default auto select platform $variants_platform "
}

print_build_type()
{
    echo
    echo "Select Build Type:"
    echo "   1.  consolidate (This is for debug)"
    echo "   2.  gki"
    echo
}

choose_build_type()
{
    local default_value=consolidate
    local ANSWER
    print_build_type
    echo -n "Which build type would you like? [$default_value] "
    if [[ -n "$1" ]] ; then
        ANSWER=$1
    else
        read ANSWER
    fi

    echo "$ANSWER"
    echo

    case $ANSWER in
        1)
            variants_type=consolidate
        ;;
        consolidate)
            variants_type=consolidate
        ;;
        2)
            variants_type=gki
        ;;
        gki)
            variants_type=gki
        ;;
        *)
            variants_type=consolidate
        ;;
    esac
    echo "variants_type $variants_type "
}

print_lto_type()
{
    echo
    echo "you can get more information  https://lkml.org/lkml/2020/12/8/1006"
    echo
    echo "While all developers agree that ThinLTO is a much more palatable
          experience than full LTO; our product teams prefer the excessive build
          time and memory high water mark (at build time) costs in exchange for
          slightly better performance than ThinLTO in <benchmarks that I've been
          told are important>.  Keeping support for full LTO in tree would help
          our product teams reduce the amount of out of tree code they have.  As
          long as <benchmarks that I've been told are important> help
          sell/differentiate phones, I suspect our product teams will continue
          to ship full LTO in production."
    echo
    echo "Select build LTO:"

    echo "   1.  full (need more time build but better performance)"
    echo "   2.  thin (build fast but can't use as performance test)"
    echo "   3.  none"
}

choose_lto_type()
{
    local default_value=none
    local ANSWER
    print_lto_type
    echo -n "Which would you like? [$default_value] "
    if [[ -n "$1" ]] ; then
        ANSWER=$1
    else
        read ANSWER
    fi

    echo "$ANSWER"
    echo

    case $ANSWER in
        1)
           export LTO=full
        ;;
        full)
           export LTO=full
        ;;
        2)
           export LTO=thin
        ;;
        thin)
           export  LTO=thin
        ;;
        *)
        ;;
    esac
    echo "LTO $LTO "
}

print_target_build()
{
    echo
    echo "Select build target:"
    echo "   1.  all (boot/dtbo)"
    echo "   2.  dtbo"
    echo "   3.  ko you need export like this \"export OPLUS_KO_PATH=drivers/input/touchscreen/focaltech_touch\""
    echo
}

choose_target_build()
{
    #set defaut value to enable
    local default_value=all
    local ANSWER
    print_target_build
    echo "Which would you like? [$default_value] "
    if [[ -n "$1" ]] ; then
        ANSWER=$1
    else
        read ANSWER
    fi

    echo "$ANSWER"
    echo
    export RECOMPILE_DTBO=""
    export RECOMPILE_KERNEL=""
    export OPLUS_BUILD_KO=""
    case $ANSWER in
        1)
            target_type=all
            export RECOMPILE_KERNEL="1"
        ;;
        all)
            target_type=all
            export RECOMPILE_KERNEL="1"
        ;;
        2)
            target_type=dtbo
            export RECOMPILE_DTBO="1"
        ;;
        dtbo)
            target_type=dtbo
            export RECOMPILE_DTBO="1"
        ;;
        3)
            export OPLUS_BUILD_KO="true"
        ;;
        ko)
            export OPLUS_BUILD_KO="true"
        ;;
        *)
            target_type=all
            export RECOMPILE_KERNEL="1"
        ;;
    esac
    echo "target_type $target_type RECOMPILE_KERNEL $RECOMPILE_KERNEL OPLUS_BUILD_KO $OPLUS_BUILD_KO "
}

print_repack_img()
{
    echo
    echo "Select enable repack boot vendor_boot img:"
    echo "   1.  enable"
    echo "   2.  disable"
    echo
}

choose_repack_img()
{
    local default_value=enable
    local ANSWER
    print_repack_img
    echo -n "Which would you like? [$default_value] "
    if [[ -n "$1" ]] ; then
        ANSWER=$1
    else
        read ANSWER
    fi

    echo "$ANSWER"
    echo

    case $ANSWER in
        1)
            export REPACK_IMG=true
        ;;
        enable)
            export REPACK_IMG=true
        ;;
        2)
            export REPACK_IMG=false
        ;;
        disable)
            export REPACK_IMG=false
        ;;
        *)
            export REPACK_IMG=true
        ;;
    esac
    echo "REPACK_IMG $REPACK_IMG "
}
choose_platform $1
choose_build_type $2
choose_lto_type $3
choose_target_build $4
choose_repack_img $5
