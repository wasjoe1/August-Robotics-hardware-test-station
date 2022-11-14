echo $0
seq=0
if [ $1 ]; then
    resort_folder=$(ls -dF $1)
    echo $resort_folder
    echo "Resorting folder: $resort_folder"
    changelog_path=$resort_folder/changelog.txt
    echo "Changelog saved at: $changelog_path"
    date >> $changelog_path
    for i in $(ls -tF $resort_folder | grep /$)
    do
        echo "Renaming $i to last_$seq" >> $changelog_path
        mv "$resort_folder$i" "$resort_folder$(expr last_$seq)/"
        # echo "$resort_folder$(expr last_$seq)/"
        seq=$(expr $seq + 1)
    done
    cat $changelog_path
fi
