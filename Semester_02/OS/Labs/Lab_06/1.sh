if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

DIR=$1

if [ ! -d "$DIR" ]; then
    echo "Error: Directory $DIR does not exist."
    exit 1
fi

sum=0

for file in $(find $DIR -type f); do
    fl=$(du $file | cut -f1)
    sum=$((sum + fl))
done

# we can instead use du -c $DIR | tail -n 1 | cut -f1
echo " size of  $DIR is $sum bytes."
