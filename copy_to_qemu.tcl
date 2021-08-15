connect -url tcp:localhost:1440
proc copy_file {src dst} {
    if {[file exists $src]} {
        tfile copy -from-host $src $dst
    } else {
        puts "Warning: $src not found\n"
    }
}

foreach {src dst} $filelist {
    copy_file $src $dst
}