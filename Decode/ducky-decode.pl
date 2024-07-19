#!/usr/bin/perl
=head1 NAME

ducky-decode.pl - A script to reverse Ducky binaries (inject.bin) into ducky-scripts

=head1 DESCRIPTION

This perl script will enumerate reverse Ducky binaries (inject.bin) into ducky-scripts.

=head1 USAGE

Usage:  ./ducky-decode.pl [-h] <-f file>

        [-h]            this help message
	[-l]		language, supported US(default), UK
	[-f]		ducky binary file

=head1 AUTHOR

Copyright (c) 01-01-2012 Midnitesnake

=cut

=head1 LICENSE

 ducky-decode - Reverse ducky binaries into ducky-scripts
 Copyright (c) 2012  Midnitesnake

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

=cut

use Getopt::Std;
$file="inject.bin";
$VERSION='0.16';

getopt('f:l:h', \%opts);

if (exists $opts{h}){ &usage;}

if (exists $opts{l}){
	$lang=$opts{l};
}else{
	$lang="US";
}
if (exists $opts{f}){
	$file=$opts{f};
}

open(FILE, "<$file") or die "Can't open ".$file." file for reading!\n\n $0 -h for help!\n\n";
binmode(FILE); 

@buffer=[500];
my ($buf, $data, $n); 
while (($n = read FILE, $data, 2) != 0) { 
	push(@buffer,$data);
} 
close(FILE);

for($i=1;$i<@buffer;$i++){
	$hex = unpack("h*",$buffer[$i]);
	$hex=~ s/00ff/EXT/g;
	if($hex eq "EXT"){
		#print "x:";
		$i++;
		$hex = unpack("h*",$buffer[$i]);
                $hex=~s/00d2/\nDELAY 300\n/g;
                $hex=~s/0019/\nDELAY 400\n/g;
		$hex=~s/005f/\nDELAY 500\n/g;
		$hex=~ s/00ff/EXT/g;
		if($hex eq "EXT"){
                	$i++;
                	$hex = unpack("h*",$buffer[$i]);
                	$hex=~s/00a5/\nDELAY 600\n/g;
			$hex=~s/^00eb/\nDELAY 700\n/g;
			$hex=~ s/00ff/EXT/g;
			if($hex eq "EXT"){
                        	$i++;
				$hex = unpack("h*",$buffer[$i]);
				$hex=~s/0032/\nDELAY 800\n/g;
				$hex=~s/0078/\nDELAY 900\n/g;
                		$hex=~s/00be/\nDELAY 1000\n/g;
				$hex=~ s/00ff/EXT/g;
				if($hex eq "EXT"){
                                	$i++;
                                	$hex = unpack("h*",$buffer[$i]);
					$hex=~s/0005/\nDELAY 1100\n/g;
					$hex=~s/004b/\nDELAY 1200\n/g;
					$hex=~ s/00ff/EXT/g;
					if($hex eq "EXT"){
						$i++;
						$hex = unpack("h*",$buffer[$i]);
						$hex=~s/007d/\nDELAY 2000\n/g;
					}
				}
			}

    		}
	}                
	$hex=~s/^0082/\nDELAY 40\n/g;
	$hex=~s/^0023/\nDELAY 50\n/g;
        $hex=~s/^00b4/\nDELAY 75\n/g;
	$hex=~s/^0069/\nDELAY 150\n/g;
        $hex=~s/^9210/\nCONTROL ESCAPE\n/g;
	$hex=~ s/^7080/\nGUI D\n/g;
        $hex=~ s/^5180/\nGUI R\n/g;
	$hex=~ s/^3e00/\nWINDOWS\n/g;
        $hex=~ s/^5600/\nMENU\n/g;
	$hex=~ s/^b200/\nTAB\n/g;
	$hex=~ s/^9300/\nCAPSLOCK\n/g;
	$hex=~ s/^1220/\$/g;
	$hex=~ s/^1320/\|/g;
	$hex=~ s/^e220/\+/g; 
	$hex=~ s/^d200/\-/g;
	$hex=~ s/^9900/\`/g;
	$hex=~ s/^4300/\'/g;
	$hex=~ s/^e120/\!/g;
	$hex=~ s/^3220/\^/g;
	$hex=~ s/^4220/\&/g;
	$hex=~ s/^5220/\*/g;
	$hex=~ s/^f220/\{/g;
	$hex=~ s/^0320/\}/g;
	$hex=~ s/^f200/\[/g;
	$hex=~ s/^0300/\]/g;
	$hex=~ s/^6220/\(/g;
        $hex=~ s/^7220/\)/g;
	$hex=~ s/^e200/\=/g;
	$hex=~ s/^6300/\,/g;
	$hex=~ s/^2220/\%/g;
        $hex=~ s/^8320/\?/g; 
        $hex=~ s/^4620/\|/g;
	$hex=~ s/^d220/_/g;
        $hex=~ s/^902e/\nALT F\n/g;
	$hex=~ s/^c140/\nALT Y\n/g;
	$hex=~ s/^6110/\nCTRL S\n/g;
	$hex=~ s/^9110/\nCTRL V\n/g;
	$hex=~ s/^b110/\nCTRL X\n/g;
	$hex=~ s/^6010/\nCTRL C\n/g;
        $hex=~ s/^d110/\nCTRL Z\n/g;
        $hex=~ s/^c200/\nSPACE\n/g;
        $hex=~ s/^9200/\nESCAPE\n/g;
        $hex=~ s/^6400/\nPRINTSCREEN\n/g;
        $hex=~ s/^9400/\nINSERT\n/g;
        $hex=~ s/^a400/\nHOME\n/g;
        $hex=~ s/^b400/\nPAGEUP\n/g;
        $hex=~ s/^c400/\nDELETE\n/g;
        $hex=~ s/^d400/\nEND\n/g;
        $hex=~ s/^e400/\nPAGEDOWN\n/g;
        $hex=~ s/^0046/\nDELAY 100\n/g;
        $hex=~ s/^008c/\nDELAY 200\n/g;
        $hex=~ s/^8200/\nENTER\n/g;
	$hex=~ s/^0500/\nLEFTARROW\n/g;
	$hex=~ s/^1500/\nDOWNARROW\n/g;
	$hex=~ s/^f400/\nRIGHTARROW\n/g;
	$hex=~ s/^2500/\nUPARROW\n/g;
	$hex=~ s/^7400/\nSCROLLLOCK\n/g;
        $hex=~ s/^3320/:/g;
	$hex=~ s/^3300/\;/g; 
	$hex=~ s/^6320/\</g;
	$hex=~ s/^7320/\>/g;
        $hex=~ s/^7300/\./g;
        $hex=~ s/^8300/\//g;
        $hex=~ s/^4000/a/g;
        $hex=~ s/^4020/A/g;
        $hex=~ s/^5000/b/g;
        $hex=~ s/^5020/B/g;
        $hex=~ s/^6000/c/g;
	$hex=~ s/^6020/C/g;
        $hex=~ s/^7000/d/g;
        $hex=~ s/^7020/D/g;
        $hex=~ s/^8000/e/g;
        $hex=~ s/^8020/E/g;
        $hex=~ s/^9000/f/g;
        $hex=~ s/^9020/F/g;
        $hex=~ s/^a000/g/g;
        $hex=~ s/^a020/G/g;
        $hex=~ s/^b000/h/g;        
	$hex=~ s/^b020/H/g;
        $hex=~ s/^c000/i/g;
        $hex=~ s/^c020/I/g;
        $hex=~ s/^d000/j/g;
        $hex=~ s/^d020/J/g;
        $hex=~ s/^e000/k/g;
        $hex=~ s/^e020/K/g;
        $hex=~ s/^f000/l/g;
        $hex=~ s/^f020/L/g;
        $hex=~ s/^0100/m/g;
        $hex=~ s/^0120/M/g;
        $hex=~ s/^1100/n/g;
        $hex=~ s/^1120/N/g;
        $hex=~ s/^2100/o/g;
        $hex=~ s/^2120/O/g;
        $hex=~ s/^3100/p/g;
        $hex=~ s/^3120/P/g;
        $hex=~ s/^4100/q/g;
        $hex=~ s/^4120/Q/g;
        $hex=~ s/^5100/r/g;
        $hex=~ s/^5120/R/g;
        $hex=~ s/^6100/s/g;
        $hex=~ s/^6120/S/g;
        $hex=~ s/^7100/t/g;
        $hex=~ s/^7120/T/g;
        $hex=~ s/^8100/u/g;
        $hex=~ s/^8120/U/g;
        $hex=~ s/^9100/v/g;
        $hex=~ s/^9120/V/g;
        $hex=~ s/^a100/w/g;
        $hex=~ s/^a120/W/g;
        $hex=~ s/^b100/x/g;
        $hex=~ s/^b120/X/g;
        $hex=~ s/^c100/y/g;
        $hex=~ s/^c120/Y/g;
        $hex=~ s/^d100/z/g;
        $hex=~ s/^d120/Z/g;

        $hex=~ s/^e100/1/g;
        $hex=~ s/^f100/2/g;
        $hex=~ s/^0200/3/g;
        $hex=~ s/^1200/4/g;
        $hex=~ s/^2200/5/g;
        $hex=~ s/^3200/6/g;
        $hex=~ s/^4200/7/g;
        $hex=~ s/^5200/8/g;
        $hex=~ s/^6200/9/g;
        $hex=~ s/^7200/0/g;

	$hex=~ s/^a300/\nF1\n/g;
        $hex=~ s/^b300/\nF2\n/g;
        $hex=~ s/^c300/\nF3\n/g;
        $hex=~ s/^d300/\nF4\n/g;
        $hex=~ s/^e300/\nF5\n/g;
        $hex=~ s/^f300/\nF6\n/g;
        $hex=~ s/^0400/\nF7\n/g;
        $hex=~ s/^1400/\nF8\n/g;
        $hex=~ s/^2400/\nF9\n/g;
        $hex=~ s/^3400/\nF10\n/g;
	$hex=~ s/^4400/\nF11\n/g;
	$hex=~ s/^5400/\nF12\n/g;
	$hex=~ s/^d32e/\nALT F4\n/g;

	if($lang eq "UK"){
	    #$hex=~ s/^1300/\\/g;
	    $hex=~ s/^f120/\"/g;
	    $hex=~ s/^4320/\@/g;
	    $hex=~ s/^0220/\£/g;
	    $hex=~ s/^4600/\\/g;
	    $hex=~ s/^5320/\~/g;
	    $hex=~ s/^5300/\#/g;
	}else{
	    $hex=~ s/^f120/\@/g;
	    $hex=~ s/^4320/\"/g;
	    $hex=~ s/^0220/\#/g;
	    $hex=~ s/^1300/\\/g;
	    $hex=~ s/^5320/\~/g;
	    $hex=~ s/^5300/\#/g;
	}

	print $hex." ";
}

sub usage{
	print "Ducky-reverse.pl version $VERSION\n";
        print "Usage: \t$0  [-h] <-f file>\n\n";
	print "\t[-l]\t\tlanguage, supported US(default), UK\n";
        print "\t[-h]\t\tthis help message\n";
        print "\t[-f]\t\tducky binary file\n";
	exit(0);
}
