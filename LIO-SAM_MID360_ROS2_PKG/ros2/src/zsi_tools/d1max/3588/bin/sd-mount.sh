#!/bin/bash
set -e

DEVNAME="$1"        # mmcblk1p1
ACTION="$2"

DEV="/dev/$DEVNAME"
#MOUNT_BASE="/mnt/"
#MOUNT_POINT="$MOUNT_BASE/$DEVNAME"
#MOUNT_POINT="$MOUNT_BASE/sdcard"
MOUNT_POINT="/ftp/tf"

log() {
    logger -t sd-mount "$*"
}

if [ "$ACTION" = "add" ]; then
    mkdir -p "$MOUNT_POINT"

    # 等待设备节点真正可用
    for i in {1..10}; do
        [ -b "$DEV" ] && break
        sleep 0.2
    done

    # 已挂载就不重复挂
    if mountpoint -q "$MOUNT_POINT"; then
        log "$DEV already mounted"
        exit 0
    fi

    mount "$DEV" "$MOUNT_POINT" && \
        log "mounted $DEV at $MOUNT_POINT"

elif [ "$ACTION" = "remove" ]; then
    if mountpoint -q "$MOUNT_POINT"; then
        if umount "$MOUNT_POINT"; then
            log "unmounted $MOUNT_POINT"
        else
            log "failed to unmount $MOUNT_POINT"
            exit 0
        fi
    fi

    # 如果目录存在且为空，则删除
    if [ -d "$MOUNT_POINT" ]; then
        if [ -z "$(ls -A "$MOUNT_POINT" 2>/dev/null)" ]; then
            rmdir "$MOUNT_POINT" && \
                log "removed empty mount dir $MOUNT_POINT"
        else
            log "$MOUNT_POINT not empty, keep it"
        fi
    fi
fi

