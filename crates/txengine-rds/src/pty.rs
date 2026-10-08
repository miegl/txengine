//! Programme type names (EN 50067 Annex F, and the RBDS variant).

/// European (RDS) programme type names, codes 0-31.
pub const RDS: [&str; 32] = [
    "None",
    "News",
    "Current affairs",
    "Information",
    "Sport",
    "Education",
    "Drama",
    "Culture",
    "Science",
    "Varied",
    "Pop music",
    "Rock music",
    "Easy listening",
    "Light classical",
    "Serious classical",
    "Other music",
    "Weather",
    "Finance",
    "Children's programmes",
    "Social affairs",
    "Religion",
    "Phone-in",
    "Travel",
    "Leisure",
    "Jazz music",
    "Country music",
    "National music",
    "Oldies music",
    "Folk music",
    "Documentary",
    "Alarm test",
    "Alarm",
];

/// North American (RBDS) programme type names, codes 0-31.
pub const RBDS: [&str; 32] = [
    "None",
    "News",
    "Information",
    "Sports",
    "Talk",
    "Rock",
    "Classic rock",
    "Adult hits",
    "Soft rock",
    "Top 40",
    "Country",
    "Oldies",
    "Soft",
    "Nostalgia",
    "Jazz",
    "Classical",
    "Rhythm and blues",
    "Soft rhythm and blues",
    "Language",
    "Religious music",
    "Religious talk",
    "Personality",
    "Public",
    "College",
    "Unassigned",
    "Unassigned",
    "Unassigned",
    "Unassigned",
    "Unassigned",
    "Weather",
    "Emergency test",
    "Emergency",
];

/// Parse a PTY given as a number or as a (case-insensitive) RDS name.
pub fn parse(s: &str) -> Option<u8> {
    let s = s.trim();
    if let Ok(n) = s.parse::<u8>() {
        return (n <= 31).then_some(n);
    }
    RDS.iter().position(|n| n.eq_ignore_ascii_case(s)).map(|i| i as u8)
}
