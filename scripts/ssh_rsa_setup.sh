#!/bin/bash

RSA=~/.ssh/id_rsa

if test -f "$RSA"; then
    ssh-keygen -q -t rsa -f ~/.ssh/id_rsa -N '' 2> /dev/null <<< y > /dev/null
fi
ssh-copy-id -i ~/.ssh/id_rsa.pub racecar@192.168.1.$1
