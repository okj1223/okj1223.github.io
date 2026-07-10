#!/usr/bin/env ruby
# frozen_string_literal: true

require "date"
require "pathname"
require "stringio"
require "yaml"

ROOT = Pathname(__dir__).join("..").realpath
PROJECTS_DIR = ROOT.join("_projects")
PROJECT_FILTERS_FILE = ROOT.join("_data", "project_filters.yml")

def blank?(value)
  return true if value.nil?
  return value.strip.empty? if value.is_a?(String)
  return value.empty? if value.respond_to?(:empty?)

  false
end

def present?(value)
  !blank?(value)
end

def load_yaml(contents, context)
  YAML.safe_load(contents, permitted_classes: [Date, Time], aliases: true)
rescue Psych::Exception => e
  raise "#{context}: invalid YAML (#{e.message})"
end

def load_front_matter(path)
  contents = path.read
  match = contents.match(/\A---\s*\n(.*?)\n---\s*\n/m)
  raise "#{path.basename}: missing YAML front matter" unless match

  load_yaml(StringIO.new(match[1]), path.basename.to_s) || {}
end

def load_project_filters
  data = load_yaml(PROJECT_FILTERS_FILE.read, PROJECT_FILTERS_FILE.basename.to_s) || []

  unless data.is_a?(Array)
    raise "#{PROJECT_FILTERS_FILE.basename}: expected an array of filter definitions"
  end

  keys = data.map do |item|
    unless item.is_a?(Hash) && present?(item["key"])
      raise "#{PROJECT_FILTERS_FILE.basename}: each filter must define a `key`"
    end

    item["key"].to_s
  end

  if keys.uniq.length != keys.length
    raise "#{PROJECT_FILTERS_FILE.basename}: duplicate filter keys detected"
  end

  keys.freeze
end

def normalized_array(value)
  Array(value).flatten.compact.reject { |item| blank?(item) }
end

def add_error(errors, message)
  errors << message
end

def validate_collection(errors, project_file, label, items, required_keys)
  unless items.is_a?(Array)
    add_error(errors, "#{project_file}: #{label} must be an array")
    return
  end

  if items.empty?
    add_error(errors, "#{project_file}: #{label} must not be empty")
    return
  end

  items.each_with_index do |item, index|
    unless item.is_a?(Hash)
      add_error(errors, "#{project_file}: #{label} #{index + 1} must be a mapping")
      next
    end

    required_keys.each do |key|
      add_error(errors, "#{project_file}: #{label} #{index + 1} missing `#{key}`") if blank?(item[key])
    end
  end
end

projects = {}
featured_projects = []
errors = []
allowed_filters = load_project_filters

Dir[PROJECTS_DIR.join("*.md")].sort.each do |file|
  path = Pathname(file)
  front_matter = load_front_matter(path)
  permalink = front_matter["permalink"]

  if blank?(permalink)
    add_error(errors, "#{path.basename}: missing `permalink` in front matter")
    next
  end

  if projects.key?(permalink)
    add_error(errors, "#{path.basename}: duplicate permalink `#{permalink}` also used by #{projects[permalink]}")
    next
  end

  projects[permalink] = path.basename.to_s

  featured = front_matter["featured"]
  featured_order = front_matter["featured_order"]

  if front_matter.key?("featured") && ![true, false].include?(featured)
    add_error(errors, "#{path.basename}: `featured` must be true or false")
  end

  if featured == true
    if !featured_order.is_a?(Integer) || featured_order <= 0
      add_error(errors, "#{path.basename}: featured projects need a positive integer `featured_order`")
    else
      featured_projects << [featured_order, path.basename.to_s]
    end
  elsif present?(featured_order)
    add_error(errors, "#{path.basename}: `featured_order` requires `featured: true`")
  end

  filter_categories = normalized_array(front_matter["filter_categories"])
  if filter_categories.empty?
    add_error(errors, "#{path.basename}: missing `filter_categories`")
  end

  unknown_filters = filter_categories - allowed_filters
  unless unknown_filters.empty?
    formatted = unknown_filters.map { |item| "`#{item}`" }.join(", ")
    add_error(errors, "#{path.basename}: unknown filter categories #{formatted}")
  end

  if filter_categories.length != filter_categories.uniq.length
    add_error(errors, "#{path.basename}: duplicate filter categories detected")
  end

  title = front_matter["card_title"] || front_matter["title"]
  add_error(errors, "#{path.basename}: missing title/card_title fallback") if blank?(title)

  description = front_matter["card_description"] ||
                front_matter["share-description"] ||
                front_matter["description"]
  add_error(errors, "#{path.basename}: missing description/card_description fallback") if blank?(description)

  add_error(errors, "#{path.basename}: missing `category_label`") if blank?(front_matter["category_label"])

  validate_collection(errors, path.basename.to_s, "impacts", front_matter["impacts"], %w[value label])
  validate_collection(errors, path.basename.to_s, "tech_tags", front_matter["tech_tags"], %w[label style])

  has_media = present?(front_matter["video_url"]) || present?(front_matter["thumbnail-img"])
  add_error(errors, "#{path.basename}: missing `video_url` or `thumbnail-img`") unless has_media
end

if featured_projects.length != 3
  add_error(errors, "expected exactly 3 featured projects, found #{featured_projects.length}")
end

featured_orders = featured_projects.map(&:first)
duplicate_featured_orders = featured_orders.group_by(&:itself).select { |_order, items| items.length > 1 }.keys
unless duplicate_featured_orders.empty?
  add_error(errors, "duplicate featured project orders: #{duplicate_featured_orders.sort.join(', ')}")
end

expected_featured_orders = (1..featured_projects.length).to_a
unless featured_orders.sort == expected_featured_orders
  add_error(errors, "featured project orders must be contiguous: #{expected_featured_orders.join(', ')}")
end

if errors.any?
  warn "Project metadata validation failed:"
  errors.each { |message| warn "- #{message}" }
  exit 1
end

puts "Validated #{projects.length} projects with project-level card metadata."
