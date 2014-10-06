#!/usr/bin/perl

use strict;
use warnings;
use IO::File;
use Getopt::Long;

my $header = 'clocks.h';
my $basediv = 1024;
my $epsilon = 0.33;
my @freqs = (1000000, 1200000, 8000000, 9600000, 16000000, 20000000);
my @divs = (1024, 256, 64, 8, 1);
GetOptions(
	"output=s" => \$header,
	"basediv=i" => \$basediv,
	"epsilon=f" => \$epsilon,
	"freq=i" => \@freqs,
	"div=i" => \@divs,
) or do {
	print("Usage: genclocks.pl [--output <output.h>] [--basediv <divider>] [--epsilon <epsilon>] [--freq <frequency>] [--div <divider>]\n");
	print("Generate clock counter values for microcontroller timing routines\n");
	print("Copyright (c) 2012-2014 Gregor Riepl <onitake\@gmail.com>\n");
	print("--output   Output file (default: $header)\n");
	print("--basediv  Base divider, i.e. divider to try first before checking all the others (default: $basediv)\n");
	print("--epsilon  Jitter percentage, specified as a +/- fraction of the clock (default: $epsilon)\n");
	print("--freq     Add to the frequencies to generate for, specified in Hz, one option for each frequency (default: " . join(", ", @freqs) . ")\n");
	print("--div      Add to the divider list, one option for each divider (default: " . join(", ", @divs) . ")\n");
	exit(1);
};

my $clocks = [
	{ name => 'SIRC_START', usecs => 2400 },
	{ name => 'SIRC_LONG', usecs => 1200 },
	{ name => 'SIRC_SHORT', usecs => 600 },
	{ name => 'SIRC_ACK', usecs => 600 },
	{ name => 'NEC_START', usecs => 9000 },
	{ name => 'NEC_ACK', usecs => 480 },
	{ name => 'NEC_MODE_CMD', usecs => 4600 },
	{ name => 'NEC_MODE_REP', usecs => 2300 },
	{ name => 'NEC_LONG', usecs => 1760 },
	{ name => 'NEC_SHORT', usecs => 640 },
	{ name => 'TIMEOUT_START', usecs => 12000 },
	{ name => 'TIMEOUT_SIRC', usecs => 1700 },
	{ name => 'TIMEOUT_NEC', usecs => 2200 },
	{ name => 'WAIT_1MS', usecs => 1000, nominmax => 1 },
	{ name => 'WAIT_10MS', usecs => 10000, nominmax => 1 },
	#{ name => 'WAIT_20MS', usecs => 20000, nominmax => 1 },
	#{ name => 'WAIT_25MS', usecs => 25000, nominmax => 1 },
	#{ name => 'WAIT_30MS', usecs => 30000, nominmax => 1 },
	#{ name => 'WAIT_50MS', usecs => 50000, nominmax => 1 },
	#{ name => 'WAIT_100MS', usecs => 100000, nominmax => 1 },
];

my $out = IO::File->new($header, 'w');

print($out <<HEADER
/*
 * This file will be overwritten by genclocks.pl.
 * Make a backup copy if you need to tune any values.
 */

#ifndef _CLOCKS_H
#define _CLOCKS_H

HEADER
);

print ($out '#');
for my $freq (@freqs) {
	print($out "if (F_CPU == $freq)\n");
	my $done = 0;
	for my $div ($basediv, @divs) {
		my $state = entry($out, $freq, $div, $epsilon, $clocks);
		if ($state) {
			print($out "#define CLOCK_DIV $div\n");
			if ($state == 2) {
				print($out "#define CLOCK_16BIT 1\n");
			}
			$done = 1;
			last;
		}
	}
	if (!$done) {
		print($out "#warning No suitable clock divider found. Please increase your clock frequency.\n");
	}
	print($out "#el");
}
print($out <<FOOTER
se
#warning CPU clock frequency not supported. Please run genclocks --freq <Your F_CPU> or add entries to clocks.h yourself.
#endif

#endif /*_CLOCKS_H*/
FOOTER
);

$out->close();

sub entry {
	my ($out, $freq, $div, $epsilon, $clocks) = @_;
	my @rclocks = @{$clocks};
	my $sixteen = 0;
	for my $clock (@rclocks) {
		my $clk = $freq * $clock->{usecs} / $div / 1000000;
		$clock->{clock} = int($clk);
		if ($clock->{nominmax}) {
			$sixteen = 1 if ($clock->{clock} > 255);
			return 0 if ($clock->{clock} == 0 || $clock->{clock} > 65535);
		} else {
			$clock->{clockmin} = int($clk * (1.0 - $epsilon));
			$clock->{clockmax} = int($clk * (1.0 + $epsilon));
			$sixteen = 1 if ($clock->{clockmax} > 255);
			return 0 if ($clock->{clock} == 0 || $clock->{clockmin} < 0 || $clock->{clockmax} > 65535);
		}
	}
	for my $clock (@rclocks) {
		print($out "#define CLOCK_$clock->{name} $clock->{clock}\n");
		print($out "#define CLOCK_$clock->{name}_MIN $clock->{clockmin}\n") if (defined($clock->{clockmin}));
		print($out "#define CLOCK_$clock->{name}_MAX $clock->{clockmax}\n") if (defined($clock->{clockmax}));
	}
	return 2 if ($sixteen);
	return 1;
}
