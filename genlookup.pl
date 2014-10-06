#!/usr/bin/perl

use strict;
use warnings;
use IO::File;
use Getopt::Long;

my $header = undef;
my $values = 32;
my $steps = 256;
my $table = 'PWM_LOOKUP';
my $function = '$input**1.5';
GetOptions(
	"output=s" => \$header,
	"values=i" => \$values,
	"steps=i" => \$steps,
	"table=s" => \$table,
	"function=s" => \$function,
) or do {
	print("Usage: genlookup.pl [--output <output.h>] [--values <values>] [--steps <steps>] [--table <name>] [--function <function>]\n");
	print("Generate lookup tables for PWM DT curves\n");
	print("Copyright (c) 2014 Gregor Riepl <onitake\@gmail.com>\n");
	print("--output   Output file (default: stdout)\n");
	print("--values   Number of scaler input values (default: $values)\n");
	print("--steps    Number of PWM steps (default: $steps)\n");
	print("--table    Name of lookup table (default: $table)\n");
	print("--function Transfer function, \$input=0..1, result should be 0..1 (default: '$function')\n");
	exit(1);
};

my $out = defined($header) ? IO::File->new($header, 'w') : \*STDOUT || die("Can't open output file");

print($out "const PROGMEM uint8_t $table\[$values] = { ");

for (my $value = 0; $value < $values; $value++) {
	my $input = $value / $values;
	my $result = eval($function);
	my $step = int($result * $steps);
	print($out "$step, ");
}

print($out "};\n");