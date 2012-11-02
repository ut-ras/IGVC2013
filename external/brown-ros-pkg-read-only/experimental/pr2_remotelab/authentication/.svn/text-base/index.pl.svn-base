#!/usr/bin/env perl
require "chall.pl";

#BEGIN {
#    my $base_module_dir = (-d '/home/brownrob/perl' ? '/home/brownrob/perl' : ( getpwuid($>) )[7] . '/perl/');
#    unshift @INC, map { $base_module_dir . $_ } @INC;
#}

use Crypt::SSLeay;
use LWP::UserAgent;
use CGI;
use Fcntl qw(:flock);

my $calendarURL = 'http://security.brown-robotics.org/current.php';

sub determineDuration {
	my $email = shift;
	my $ip = shift;

	my $duration = 0;

	my $ua = LWP::UserAgent->new;
	$ua->agent('Mozilla/5.0');
	my $response = $ua->get($calendarURL . '?jsonp=invoke&q=rosjs-session')->content;
	if ($response =~ /.*$email",(\d+)/) {
		$duration = $1;
	}

	return $duration;
}

my $q = CGI->new;

my $waitTime = 300;
my $googleURL = 'https://www.google.com/accounts/o8/ud';

print $q->header('text/javascript');

if ($q->param('jsonp')) {
	if (!$q->param('confirmation') && !$q->param('key')) {
		my $challenge = &generate_random_string(8);

		open(FILE, ">challenges/$challenge");
		flock FILE, LOCK_EX;
		print FILE time();
		close FILE;

		print $q->param('jsonp') . '({"challenge":"' . $challenge . '"});';
	} elsif ($q->param('confirmation')) {
		my $confirmation = $q->unescape($q->param('confirmation'));
		my $ua = LWP::UserAgent->new;
		my $response = $ua->get($googleURL . '?' . $confirmation)->content;
		if ($response =~ m/is_valid:true/) {
				$confirmation =~ /openid.return_to=([^&]*)/;
				my $returnTo = $1;
				$returnTo = $q->unescape($returnTo);
				$returnTo =~ /challenge=([^&]*)/;
				my $challenge = $1;
				if(open(FILE, "<challenges/" . $challenge)) {
					flock FILE, LOCK_EX;
					my $elapsed = time() - <FILE>;
					close FILE;
					if ($elapsed <= $waitTime) {
						unlink("challenges/" . $challenge);
						$confirmation =~ /openid\.ext1\.value\.email=([^&]*)/;
						my $email = $q->unescape($1);
						my $ip = $ENV{'REMOTE_ADDR'};
						open(FILE, ">>log/access.txt");
						flock FILE, LOCK_EX;
						my @now = localtime(time());
						print FILE sprintf("%02d",$now[3]) . '/' . sprintf("%02d",$now[4]) . '/' . (1900+$now[5]) . "\t" . $now[2] . ':' . $now[1] . ':' . $now[0] . "\t" . $email . ',' . $ip . "\n";
						close FILE;
						my $key = &generate_random_string(8);
						open(FILE, ">keys/$key");
						flock FILE, LOCK_EX;
						print FILE "$email\n";
						close FILE;
						print $q->param('jsonp') . '({"valid":true,"key":"' . $key . '"});';
					} else {
						$confirmation =~ /openid\.ext1\.value\.email=([^&]*)/;
						my $email = $q->unescape($1);
						my $ip = $ENV{'REMOTE_ADDR'};
						open(FILE, ">>log/error.txt");
						flock FILE, LOCK_EX;
						my @now = localtime(time());
						print FILE sprintf("%02d",$now[3]) . '/' . sprintf("%02d",$now[4]) . '/' . (1900+$now[5]) . "\t" . $now[2] . ':' . $now[1] . ':' . $now[0] . "\t" . $email . ',' . $ip . "\n";
						close FILE;
						print $q->param('jsonp') . '({"valid":false});';
					}
				} else {
					print $q->param('jsonp') . '({"valid":false});';
				}
		} else {
				print $q->param('jsonp') . '({"valid":false});';
		}
	} elsif ($q->param('key')) {
		my $left = 0;
		if (open(FILE, "<keys/" . $q->param('key'))) {
			flock FILE, LOCK_EX;
			my $email = <FILE>;
			chomp($email);
			close FILE;
			my $ip = $ENV{'REMOTE_ADDR'};
			$left = &determineDuration($email,$ip);
		}
		print $q->param('jsonp') . '({"valid":true,"left":"' . $left . '","key":"' . $q->param('key') . '"});';
	} else {
		print $q->param('jsonp') . '({"reply":"ERROR","valid":false});';
	}
} else {
	print "//bad call";
}
